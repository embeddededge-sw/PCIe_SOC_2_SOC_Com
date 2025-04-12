

#include <linux/etherdevice.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/pci-epc.h>
#include <linux/pci-epf.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/pci_ids.h>
#include <linux/pci_regs.h>
#include "tegra_vnet.h"

#define PCI_DEVICE_ID_TI_AM64				0xb010

#define BAR0_SIZE SZ_4M

enum bar0_amap_type {
	META_DATA,
	SIMPLE_IRQ,
	DMA_IRQ = SIMPLE_IRQ,
	EP_MEM,
	HOST_MEM,
	HOST_DMA,
	EP_RX_BUF,
	AMAP_MAX,
};

struct bar0_amap {
	int size;
	struct page *page;
	void *virt;
	dma_addr_t iova;
	dma_addr_t phy;
};

struct irqsp_data {
        struct nvhost_interrupt_syncpt *is;
        struct work_struct reprime_work;
        struct device *dev;
};

struct pci_epf_tvnet {
        struct pci_epf *epf;
        struct device *fdev;
        struct pci_epf_header header;
		
        void __iomem *dma_base;
        struct bar0_amap bar0_amap[AMAP_MAX];
        struct bar_md *bar_md;
        dma_addr_t bar0_iova;
        struct net_device *ndev;
        struct napi_struct napi;
        bool pcie_link_status;
        struct ep_ring_buf ep_ring_buf;
        struct host_ring_buf host_ring_buf;
        enum dir_link_state tx_link_state;
        enum dir_link_state rx_link_state;
        enum os_link_state os_link_state;
		
        /* To synchronize network link state machine*/
        struct mutex link_state_lock;
        wait_queue_head_t link_state_wq;
        struct list_head h2ep_empty_list;
#if ENABLE_DMA
        struct dma_desc_cnt desc_cnt;
#endif
        /* To protect h2ep empty list */
        spinlock_t h2ep_empty_lock;
        dma_addr_t rx_buf_iova;
        unsigned long *rx_buf_bitmap;
        int rx_num_pages;
        void __iomem *tx_dst_va;
        phys_addr_t tx_dst_pci_addr;
        void *ep_dma_virt;
        dma_addr_t ep_dma_iova;
        struct irqsp_data *ctrl_irqsp;
        struct irqsp_data *data_irqsp;

        struct am64_vnet_counter h2ep_ctrl;
        struct am64_vnet_counter ep2h_ctrl;
        struct am64_vnet_counter h2ep_empty;
        struct am64_vnet_counter h2ep_full;
        struct am64_vnet_counter ep2h_empty;
        struct am64_vnet_counter ep2h_full;
};

static int tvnet_ep_pci_epf_bind(struct pci_epf *epf)
{
	struct pci_epf_tvnet *tvnet = epf_get_drvdata(epf);
	struct pci_epc *epc = epf->epc;

	struct device *fdev = &epf->dev;
	struct device *cdev = epc->dev.parent;
	struct iommu_domain *domain = iommu_get_domain_for_dev(cdev);
	struct platform_device *pdev = of_find_device_by_node(cdev->of_node);
	struct ep_ring_buf *ep_ring_buf = &tvnet->ep_ring_buf;
	struct host_ring_buf *host_ring_buf = &tvnet->host_ring_buf;
	struct net_device *ndev;
	struct bar_md *bar_md;
	struct resource *res;
	struct bar0_amap *amap;
	struct tvnet_dma_desc *dma_desc;
	int ret, size, bitmap_size;
#if (LINUX_VERSION_CODE > KERNEL_VERSION(5, 14, 0))
	unsigned long shift;
#endif
	if (!domain) {
		dev_err(fdev, "IOMMU domain not found\n");
		ret = -ENXIO;
		goto fail;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "atu_dma");
	if (!res) {
		dev_err(fdev, "missing atu_dma resource in DT\n");
		ret = PTR_ERR(res);
		goto fail;
	}

	tvnet->dma_base = devm_ioremap(fdev, res->start + DMA_OFFSET,
				       resource_size(res) - DMA_OFFSET);
	if (IS_ERR(tvnet->dma_base)) {
		ret = PTR_ERR(tvnet->dma_base);
		dev_err(fdev, "dma region map failed: %d\n", ret);
		goto fail;
	}

#if (LINUX_VERSION_CODE > KERNEL_VERSION(5, 14, 0))
	tvnet->iovad = (struct iova_domain *)&domain->iova_cookie->iovad;

	shift = iova_shift(tvnet->iovad);

	tvnet->iova = alloc_iova(tvnet->iovad, BAR0_SIZE >> shift,
			cdev->coherent_dma_mask >> shift, true);
	if (!tvnet->iova) {
		dev_err(fdev, "iova region map failed\n");
		goto fail;
	}

	tvnet->bar0_iova = iova_dma_addr(tvnet->iovad, tvnet->iova);
	tvnet_get_host1x_dev(epf);
#else
	tvnet->bar0_iova = iommu_dma_alloc_iova(cdev, BAR0_SIZE,
						cdev->coherent_dma_mask);
	if (!tvnet->bar0_iova) {
		dev_err(fdev, "iommu_dma_alloc_iova() failed\n");
		ret = -ENOMEM;
		goto fail;
	}
#endif
	pr_debug("BAR0 IOVA: 0x%llx\n", tvnet->bar0_iova);

	/* BAR0 metadata memory allocation */
	tvnet->bar0_amap[META_DATA].iova = tvnet->bar0_iova;
	tvnet->bar0_amap[META_DATA].size = PAGE_SIZE;
	ret = tvnet_ep_alloc_single_page_bar0_mem(epf, META_DATA);
	if (ret < 0) {
		dev_err(fdev, "BAR0 metadata alloc failed: %d\n", ret);
		goto free_iova;
	}

	tvnet->bar_md = (struct bar_md *)tvnet->bar0_amap[META_DATA].virt;
	bar_md = tvnet->bar_md;

	/* BAR0 SIMPLE_IRQ setup: two interrupts required two pages */
	amap = &tvnet->bar0_amap[SIMPLE_IRQ];
	amap->iova = tvnet->bar0_amap[META_DATA].iova +
		tvnet->bar0_amap[META_DATA].size;
	amap->size = 2 * PAGE_SIZE;

	ret = tvnet_ep_pci_epf_setup_irqsp(tvnet);
	if (ret < 0) {
		dev_err(fdev, "irqsp setup failed: %d\n", ret);
		goto free_bar0_md;
	}

	/* BAR0 EP memory allocation */
	amap = &tvnet->bar0_amap[EP_MEM];
	amap->iova = tvnet->bar0_amap[SIMPLE_IRQ].iova +
		tvnet->bar0_amap[SIMPLE_IRQ].size;
	size = sizeof(struct ep_own_cnt) + (RING_COUNT *
		(sizeof(struct ctrl_msg) + 2 * sizeof(struct data_msg)));
	amap->size = PAGE_ALIGN(size);
	ret = tvnet_ep_alloc_multi_page_bar0_mem(epf, EP_MEM);
	if (ret < 0) {
		dev_err(fdev, "BAR0 EP mem alloc failed: %d\n", ret);
		goto free_irqsp;
	}

	ep_ring_buf->ep_cnt = (struct ep_own_cnt *)amap->virt;
	ep_ring_buf->ep2h_ctrl_msgs = (struct ctrl_msg *)
				(ep_ring_buf->ep_cnt + 1);
	ep_ring_buf->ep2h_full_msgs = (struct data_msg *)
				(ep_ring_buf->ep2h_ctrl_msgs + RING_COUNT);
	ep_ring_buf->h2ep_empty_msgs = (struct data_msg *)
				(ep_ring_buf->ep2h_full_msgs + RING_COUNT);
	/* Clear EP counters */
	memset(ep_ring_buf->ep_cnt, 0, sizeof(struct ep_own_cnt));

	/* BAR0 host memory allocation */
	amap = &tvnet->bar0_amap[HOST_MEM];
	amap->iova = tvnet->bar0_amap[EP_MEM].iova +
					tvnet->bar0_amap[EP_MEM].size;
	size = (sizeof(struct host_own_cnt)) + (RING_COUNT *
		(sizeof(struct ctrl_msg) + 2 * sizeof(struct data_msg)));
	amap->size = PAGE_ALIGN(size);
	ret = tvnet_ep_alloc_multi_page_bar0_mem(epf, HOST_MEM);
	if (ret < 0) {
		dev_err(fdev, "BAR0 host mem alloc failed: %d\n", ret);
		goto free_ep_mem;
	}

	host_ring_buf->host_cnt = (struct host_own_cnt *)amap->virt;
	host_ring_buf->h2ep_ctrl_msgs = (struct ctrl_msg *)
				(host_ring_buf->host_cnt + 1);
	host_ring_buf->ep2h_empty_msgs = (struct data_msg *)
				(host_ring_buf->h2ep_ctrl_msgs + RING_COUNT);
	host_ring_buf->h2ep_full_msgs = (struct data_msg *)
				(host_ring_buf->ep2h_empty_msgs + RING_COUNT);
	/* Clear host counters */
	memset(host_ring_buf->host_cnt, 0, sizeof(struct host_own_cnt));

	/*
	 * Allocate local memory for DMA read link list elements.
	 * This is exposed through BAR0 to initiate DMA read from host.
	 */
	amap = &tvnet->bar0_amap[HOST_DMA];
	amap->iova = tvnet->bar0_amap[HOST_MEM].iova +
					tvnet->bar0_amap[HOST_MEM].size;
	size = ((DMA_DESC_COUNT + 1) * sizeof(struct tvnet_dma_desc));
	amap->size = PAGE_ALIGN(size);
	ret = tvnet_ep_alloc_multi_page_bar0_mem(epf, HOST_DMA);
	if (ret < 0) {
		dev_err(fdev, "BAR0 host dma mem alloc failed: %d\n", ret);
		goto free_host_mem;
	}

	/* Set link list pointer to create a dma desc ring */
	memset(amap->virt, 0, amap->size);
	dma_desc = (struct tvnet_dma_desc *)amap->virt;
	dma_desc[DMA_DESC_COUNT].sar_low = (amap->iova & 0xffffffff);
	dma_desc[DMA_DESC_COUNT].sar_high = ((amap->iova >> 32) & 0xffffffff);
	dma_desc[DMA_DESC_COUNT].ctrl_reg.ctrl_e.llp = 1;

	/* Update BAR metadata region with offsets */
	/* EP owned memory */
	bar_md->ep_own_cnt_offset = tvnet->bar0_amap[META_DATA].size +
					tvnet->bar0_amap[SIMPLE_IRQ].size;
	bar_md->ctrl_md.ep2h_offset = bar_md->ep_own_cnt_offset +
					sizeof(struct ep_own_cnt);
	bar_md->ctrl_md.ep2h_size = RING_COUNT;
	bar_md->ep2h_md.ep2h_offset = bar_md->ctrl_md.ep2h_offset +
					(RING_COUNT * sizeof(struct ctrl_msg));
	bar_md->ep2h_md.ep2h_size = RING_COUNT;
	bar_md->h2ep_md.ep2h_offset = bar_md->ep2h_md.ep2h_offset +
					(RING_COUNT * sizeof(struct data_msg));
	bar_md->h2ep_md.ep2h_size = RING_COUNT;

	/* Host owned memory */
	bar_md->host_own_cnt_offset = bar_md->ep_own_cnt_offset +
					tvnet->bar0_amap[EP_MEM].size;
	bar_md->ctrl_md.h2ep_offset = bar_md->host_own_cnt_offset +
					sizeof(struct host_own_cnt);
	bar_md->ctrl_md.h2ep_size = RING_COUNT;
	bar_md->ep2h_md.h2ep_offset = bar_md->ctrl_md.h2ep_offset +
					(RING_COUNT * sizeof(struct ctrl_msg));
	bar_md->ep2h_md.h2ep_size = RING_COUNT;
	bar_md->h2ep_md.h2ep_offset = bar_md->ep2h_md.h2ep_offset +
					(RING_COUNT * sizeof(struct data_msg));
	bar_md->h2ep_md.h2ep_size = RING_COUNT;

	tvnet->h2ep_ctrl.rd = &ep_ring_buf->ep_cnt->h2ep_ctrl_rd_cnt;
	tvnet->h2ep_ctrl.wr = &host_ring_buf->host_cnt->h2ep_ctrl_wr_cnt;
	tvnet->ep2h_ctrl.rd = &host_ring_buf->host_cnt->ep2h_ctrl_rd_cnt;
	tvnet->ep2h_ctrl.wr = &ep_ring_buf->ep_cnt->ep2h_ctrl_wr_cnt;
	tvnet->h2ep_empty.rd = &host_ring_buf->host_cnt->h2ep_empty_rd_cnt;
	tvnet->h2ep_empty.wr = &ep_ring_buf->ep_cnt->h2ep_empty_wr_cnt;
	tvnet->h2ep_full.rd = &ep_ring_buf->ep_cnt->h2ep_full_rd_cnt;
	tvnet->h2ep_full.wr = &host_ring_buf->host_cnt->h2ep_full_wr_cnt;
	tvnet->ep2h_empty.rd = &ep_ring_buf->ep_cnt->ep2h_empty_rd_cnt;
	tvnet->ep2h_empty.wr = &host_ring_buf->host_cnt->ep2h_empty_wr_cnt;
	tvnet->ep2h_full.rd = &host_ring_buf->host_cnt->ep2h_full_rd_cnt;
	tvnet->ep2h_full.wr = &ep_ring_buf->ep_cnt->ep2h_full_wr_cnt;

	/* RAM region for use by host when programming EP DMA controller */
	bar_md->host_dma_offset = bar_md->host_own_cnt_offset +
					tvnet->bar0_amap[HOST_MEM].size;
	bar_md->host_dma_size = tvnet->bar0_amap[HOST_DMA].size;

	/* EP Rx pkt IOVA range */
	tvnet->rx_buf_iova = tvnet->bar0_amap[HOST_DMA].iova +
					tvnet->bar0_amap[HOST_DMA].size;
	bar_md->bar0_base_phy = tvnet->bar0_iova;
	bar_md->ep_rx_pkt_offset = bar_md->host_dma_offset +
					tvnet->bar0_amap[HOST_DMA].size;
	bar_md->ep_rx_pkt_size = BAR0_SIZE -
					tvnet->bar0_amap[META_DATA].size -
					tvnet->bar0_amap[SIMPLE_IRQ].size -
					tvnet->bar0_amap[EP_MEM].size -
					tvnet->bar0_amap[HOST_MEM].size -
					tvnet->bar0_amap[HOST_DMA].size;

	/* Create bitmap for allocating RX buffers */
	tvnet->rx_num_pages = (bar_md->ep_rx_pkt_size >> PAGE_SHIFT);
	bitmap_size = BITS_TO_LONGS(tvnet->rx_num_pages) * sizeof(long);
	tvnet->rx_buf_bitmap = devm_kzalloc(fdev, bitmap_size, GFP_KERNEL);
	if (!tvnet->rx_buf_bitmap) {
		dev_err(fdev, "rx_bitmap mem alloc failed\n");
		ret = -ENOMEM;
		goto free_host_dma;
	}

	/* Allocate PCIe memory for RP's dst address during xmit */
	tvnet->tx_dst_va = pci_epc_mem_alloc_addr(epc,
						     &tvnet->tx_dst_pci_addr,
						     SZ_64K);
	if (!tvnet->tx_dst_va) {
		dev_err(fdev, "failed to allocate dst PCIe address\n");
		ret = -ENOMEM;
		goto free_host_dma;
	}

	/* Register network device */
	ndev = alloc_etherdev(0);
	if (!ndev) {
		dev_err(fdev, "alloc_etherdev() failed\n");
		ret = -ENOMEM;
		goto free_pci_mem;
	}

	eth_hw_addr_random(ndev);
	tvnet->ndev = ndev;
	SET_NETDEV_DEV(ndev, fdev);
	ndev->netdev_ops = &tvnet_netdev_ops;
#if defined(NV_NETIF_NAPI_ADD_WEIGHT_PRESENT) /* Linux v6.1 */
	netif_napi_add_weight(ndev, &tvnet->napi, tvnet_ep_poll, TVNET_NAPI_WEIGHT);
#else
	netif_napi_add(ndev, &tvnet->napi, tvnet_ep_poll, TVNET_NAPI_WEIGHT);
#endif
	ndev->mtu = TVNET_DEFAULT_MTU;

	ret = register_netdev(ndev);
	if (ret < 0) {
		dev_err(fdev, "register_netdev() failed: %d\n", ret);
		goto fail_free_netdev;
	}
	netif_carrier_off(ndev);

	tvnet->rx_link_state = DIR_LINK_STATE_DOWN;
	tvnet->tx_link_state = DIR_LINK_STATE_DOWN;
	tvnet->os_link_state = OS_LINK_STATE_DOWN;
	mutex_init(&tvnet->link_state_lock);
	init_waitqueue_head(&tvnet->link_state_wq);

	INIT_LIST_HEAD(&tvnet->h2ep_empty_list);
	spin_lock_init(&tvnet->h2ep_empty_lock);

	INIT_WORK(&tvnet->raise_irq_work, tvnet_ep_raise_irq_work_function);

#if (LINUX_VERSION_CODE <= KERNEL_VERSION(4, 15, 0))
	/* TODO Update it to 64-bit prefetch type */
	ret = pci_epc_set_bar(epc, BAR_0, tvnet->bar0_iova, BAR0_SIZE,
			      PCI_BASE_ADDRESS_SPACE_MEMORY |
			      PCI_BASE_ADDRESS_MEM_TYPE_32);
	if (ret < 0) {
		dev_err(fdev, "pci_epc_set_bar() failed: %d\n", ret);
		goto fail_unreg_netdev;
	}
#endif

#if (LINUX_VERSION_CODE <= KERNEL_VERSION(4, 15, 0))
	ret = pci_epc_set_msi(epc, epf->msi_interrupts);
	if (ret) {
		dev_err(fdev, "pci_epc_set_msi() failed: %d\n", ret);
		goto fail_clear_bar;
	}
#endif

	/* Allocate local memory for DMA write link list elements */
	size = ((DMA_DESC_COUNT + 1) * sizeof(struct tvnet_dma_desc));
	tvnet->ep_dma_virt = dma_alloc_coherent(cdev, size,
						&tvnet->ep_dma_iova,
						GFP_KERNEL);
	if (!tvnet->ep_dma_virt) {
		dev_err(fdev, "%s ep dma mem alloc failed\n", __func__);
		ret = -ENOMEM;
		goto fail_clear_bar;
	}

	/* Set link list pointer to create a dma desc ring */
	memset(tvnet->ep_dma_virt, 0, size);
	dma_desc = (struct tvnet_dma_desc *)tvnet->ep_dma_virt;
	dma_desc[DMA_DESC_COUNT].sar_low = (tvnet->ep_dma_iova & 0xffffffff);
	dma_desc[DMA_DESC_COUNT].sar_high = ((tvnet->ep_dma_iova >> 32) &
					     0xffffffff);
	dma_desc[DMA_DESC_COUNT].ctrl_reg.ctrl_e.llp = 1;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
	nvhost_interrupt_syncpt_prime(tvnet->ctrl_irqsp->is);
	nvhost_interrupt_syncpt_prime(tvnet->data_irqsp->is);

#if (LINUX_VERSION_CODE > KERNEL_VERSION(4, 15, 0))
	epf->nb.notifier_call = tvnet_ep_pci_epf_notifier;
	pci_epc_register_notifier(epc, &epf->nb);
#endif
#endif

	return 0;

fail_clear_bar:
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(4, 15, 0))
	pci_epc_clear_bar(epc, BAR_0);
fail_unreg_netdev:
#endif
	unregister_netdev(ndev);
fail_free_netdev:
	netif_napi_del(&tvnet->napi);
	free_netdev(ndev);
free_pci_mem:
	pci_epc_mem_free_addr(epc, tvnet->tx_dst_pci_addr, tvnet->tx_dst_va,
			      SZ_64K);
free_host_dma:
	tvnet_ep_free_multi_page_bar0_mem(epf, HOST_DMA);
free_host_mem:
	tvnet_ep_free_multi_page_bar0_mem(epf, HOST_MEM);
free_ep_mem:
	tvnet_ep_free_multi_page_bar0_mem(epf, EP_MEM);
free_irqsp:
	tvnet_ep_pci_epf_destroy_irqsp(tvnet);
free_bar0_md:
	tvnet_ep_free_single_page_bar0_mem(epf, META_DATA);
free_iova:
#if (LINUX_VERSION_CODE > KERNEL_VERSION(5, 14, 0))
	__free_iova(tvnet->iovad, tvnet->iova);
#else
	iommu_dma_free_iova(cdev, tvnet->bar0_iova, BAR0_SIZE);
#endif
fail:
	return ret;
}

static void tvnet_ep_pci_epf_unbind(struct pci_epf *epf)
{
	
}

static const struct pci_epf_device_id tvnet_ep_epf_tvnet_ids[] = {
        { .name = "pci_epf_tvnet", },
        { },
};

int am64_vnet_ep_epf_vnet_probe(struct pci_epf *epf)
{
        struct device *fdev = &epf->dev;
        struct pci_epf_tvnet *tvnet;

        tvnet = devm_kzalloc(fdev, sizeof(*tvnet), GFP_KERNEL);
        if (!tvnet)
                return -ENOMEM;

        epf_set_drvdata(epf, tvnet);
        tvnet->fdev = fdev;
        tvnet->epf = epf;

		epf->event_ops = &tvnet_event_ops;
		
        tvnet->header.vendorid = PCI_VENDOR_ID_TI;
        tvnet->header.deviceid = PCI_DEVICE_ID_TI_AM64_NETWORK;
        tvnet->header.revid = 0x0;
        tvnet->header.baseclass_code = PCI_BASE_CLASS_NETWORK;
        tvnet->header.subclass_code = (PCI_CLASS_NETWORK_OTHER & 0xff);
        tvnet->header.subsys_vendor_id = PCI_VENDOR_ID_TI;
        tvnet->header.subsys_id = 0x0;
        tvnet->header.interrupt_pin = PCI_INTERRUPT_INTA;
        epf->header = &tvnet->header;

        return 0;
}

static struct pci_epf_ops am64_vnet_ep_ops = {
        .bind           = am64_vnet_ep_pci_epf_bind,
        .unbind         = am64_vnet_ep_pci_epf_unbind,     
};

static struct pci_epf_driver am64_vnet_driver = {
        .driver.name    = "pci_epf_tvnet",
        .probe          = am64_vnet_ep_epf_vnet_probe,
        .id_table       = am64_vnet_ep_epf_vnet_ids,
        .ops            = &am64_vnet_ep_ops,
        .owner          = THIS_MODULE,
};

static int __init am64_vnet_ep_pci_epf_init(void)
{
        int ret;

        ret = pci_epf_register_driver(&am64_vnet_driver);
        if (ret < 0) {
                pr_err("Failed to register EPF Tegra vnet driver: %d\n", ret);
                return ret;
        }

        return 0;
}
module_init(am64_vnet_ep_pci_epf_init);

static void __exit am64_vnet_ep_pci_epf_exit(void)
{
        pci_epf_unregister_driver(&am64_vnet_driver);
}
module_exit(am64_vnet_ep_pci_epf_exit);

MODULE_DESCRIPTION("PCI EPF TMDS64EVM VIRTUAL NETWORK DRIVER");
MODULE_AUTHOR("Mir Faisal <mirfaisalfos@gmail.com>");
MODULE_LICENSE("GPL v2");
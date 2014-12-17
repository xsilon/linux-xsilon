/*
 * FT313 on chip memory management.
 *
 * Copyright (C) 2011 Chang Yang <chang.yang@ftdichip.com>
 *
 * This code is *strongly* based on EHCI-HCD code by David Brownell since
 * the chip is a quasi-EHCI compatible.
 *
 * Licensed under GPL version 2 only.
 */

/* this file is part of ft313-hcd.c */

// Buffer memory block size must be from small to big
static struct ft313_mem_map_t g_mem_map[] = {
	{QHEAD,		sizeof(struct ehci_qh_hw),	40,	32},
	{QTD,		sizeof(struct ehci_qtd_hw),	80,	32},
	{ITD,		sizeof(struct ehci_itd_hw),	20,	32},
	{SITD,		sizeof(struct ehci_sitd_hw),	32,	32},
	{BUFFER,	64,				 8,	 1},
	{BUFFER,	256,				 4,	 1},
	{BUFFER,	512,				 2,	 1},
	{BUFFER,	1024 * 2,			 2,	 1},
	{BUFFER,	1024 * 4,			 2,	 1},
//	{BUFFER,	1024 * 16,			 1,	 1},
	{0,		0,				 0,	 0}
};

char* get_name_by_type(enum mem_blk_type type)
{
	switch(type)
	{
	case ITD:
		return "iTD";
	case QHEAD:
		return "qHEAD";
	case SITD:
		return "siTD";
	case FSTN:
		return "FSTN";
	case QTD:
		return "qTD";
	case BUFFER:
		return "Buffer";
	}

	return "Error: Wrong Type!";
}


static void
display_mm_status(struct ft313_hcd *ft313, int in_use_only)
{
	struct ft313_mem_blk *blk;
	int i;

	blk = ft313->mem;

	for (i =0; i < ft313->mem_blk_num; i++) {
		if (in_use_only == TRUE) {
			if (blk->in_use == TRUE)
					DEBUG_MSG("No.%2d. type is %s, size is %d, offset at 0x%04X\n",
							i, get_name_by_type(blk->type), blk->size, blk->offset);
		} else {
			DEBUG_MSG("No.%2d. status is %s, type is %s, size is %d, offset at 0x%04X\n",
					i, ((blk->in_use == TRUE)? "in use": "not in use"), get_name_by_type(blk->type), blk->size, blk->offset);
		}

		blk++;
	}

	blk--; // restore to the last block
	DEBUG_MSG("Reminding free memory is %d bytes\n", FT313_CHIP_MEM_AMT - (blk->offset + blk->size));
}

void display_qh_status(struct ft313_hcd *ft313, int in_use_only)
		{
	struct ft313_mem_blk *blk;
	int i;

	blk = ft313->mem;

	for (i =0; i < ft313->mem_blk_num; i++) {
		if (in_use_only == TRUE) {
			if (blk->in_use == TRUE && blk->type == QHEAD)
				DEBUG_MSG("No.%2d. type is %s, size is %d, offset at 0x%04X\n",
					  i, get_name_by_type(blk->type), blk->size, blk->offset);
		} else {
			if (blk->type == QHEAD)
			DEBUG_MSG("No.%2d. status is %s, type is %s, size is %d, offset at 0x%04X\n",
					i, ((blk->in_use == TRUE)? "in use": "not in use"), get_name_by_type(blk->type), blk->size, blk->offset);
		}

		blk++;

		if (blk->type != QHEAD)
			break;

	}
}


static int
ft313_mem_blk_init(struct ft313_hcd *ft313, gfp_t flags)
{
	struct ft313_mem_map_t *mem_map_item_ptr;
	struct ft313_mem_blk *current_blk;
	unsigned current_offset;
	int j, total_items = 0;

	mem_map_item_ptr = g_mem_map;

	while(mem_map_item_ptr->type != 0) {
		total_items += mem_map_item_ptr->num_of_items;
		mem_map_item_ptr++;
	}

	ft313->mem = kmalloc(total_items * sizeof(struct ft313_mem_blk), flags);
	ft313->mem_blk_num = total_items;

	memset(ft313->mem, 0, total_items * sizeof(struct ft313_mem_blk));

	mem_map_item_ptr = g_mem_map;
	current_offset =  ft313->periodic_size * sizeof(__le32); /* FixMe, need to init this first! */
	current_blk = ft313->mem;

	do {
		for (j = 0; j < mem_map_item_ptr->num_of_items; j++) {
			if (0 != current_offset % mem_map_item_ptr->alignment) { //Not aligned
				current_offset = (current_offset / mem_map_item_ptr->alignment + 1)
						* mem_map_item_ptr->alignment;
			}

			current_blk->type = mem_map_item_ptr->type;
			current_blk->size = mem_map_item_ptr->size;
			current_blk->offset = current_offset;

			if ((current_blk->offset + current_blk->size) > FT313_CHIP_MEM_AMT) {
				ERROR_MSG("Memory map is not valid!\n");
				goto Fail;
			}

			current_blk++;
			current_offset += mem_map_item_ptr->size;
		}

		mem_map_item_ptr++;

	} while(mem_map_item_ptr->type != 0);

//	display_mm_status(ft313, FALSE);

	return 0;
Fail:
	kfree(ft313->mem);
	return -ENOMEM;
}

struct ft313_mem_blk* allocate_mem_blk(struct ft313_hcd *ft313, unsigned type, unsigned size)
{
	int i;
	struct ft313_mem_blk *current_blk = NULL, *curr_available_blk = NULL;
	unsigned long flags = 0;

	current_blk = ft313->mem;

	if (NULL == ft313->mem) {
		printk("Memory control block not init yet!\n");
		BUG();
		return NULL;
	}

	safe_spin_lock(&ft313->mem_lock, &flags);

	if (type != BUFFER) {// Allocate discriptor
		for (i = 0; i < ft313->mem_blk_num; i++) {
			if (current_blk->type == type && current_blk->in_use == FALSE) {
				current_blk->in_use = TRUE;
				safe_spin_unlock(&ft313->mem_lock, &flags);
				return current_blk;
			}

			current_blk++;
		}
		safe_spin_unlock(&ft313->mem_lock, &flags);

		return NULL;
	} else { // Allocate memory
		for (i = 0; i < ft313->mem_blk_num; i++) {
			if (current_blk->type == type && current_blk->in_use == FALSE) {
				if (size <= current_blk->size) {
					current_blk->in_use = TRUE;
					safe_spin_unlock(&ft313->mem_lock, &flags);
					return current_blk;
				} else {
					curr_available_blk = current_blk;
				}
			}

			current_blk++;
		}

		if (curr_available_blk != NULL)
			curr_available_blk->in_use = TRUE;
		safe_spin_unlock(&ft313->mem_lock, &flags);
		return curr_available_blk; // return the last avaialble as it has largest size or NULL
	}
}

int free_mem_blk(struct ft313_hcd *ft313, unsigned offset)
{
	int i, ret = -1;
	struct ft313_mem_blk *current_blk;
	unsigned long flags = 0;

	safe_spin_lock(&ft313->mem_lock, &flags);

	current_blk = ft313->mem;

	for (i = 0; i < ft313->mem_blk_num; i++)
	{
		if (current_blk[i].offset == offset)
		{
			current_blk[i].in_use = FALSE;
			ret = 0;
			break;
		}
	}

	safe_spin_unlock(&ft313->mem_lock, &flags);
	return ret;

}

int get_mem_blk_index(struct ft313_hcd *ft313, unsigned offset)
{
	int i, ret = -1;
	struct ft313_mem_blk *current_blk;

	current_blk = ft313->mem;

	for (i = 0; i < ft313->mem_blk_num; i++)
	{
		if (current_blk[i].offset == offset)
		{
			ret = i;
			break;
		}
	}

	return ret;

}

static inline void ft313_qtd_init(struct ft313_hcd *ft313, struct ehci_qtd *qtd,
				  dma_addr_t dma)
{
	u32 qtd_ft313 = qtd->qtd_ft313; // Perserve offset in ft313

	DEBUG_MSG("qTD 0x%X is being initailized\n", qtd_ft313);
	memset (qtd, 0, sizeof *qtd);
	qtd->qtd_dma = dma;
	qtd->qtd_ft313 = qtd_ft313;
	qtd->hw_token = cpu_to_hc32(ft313,QTD_STS_HALT);
	qtd->hw_next = EHCI_LIST_END(ft313);
	qtd->hw_alt_next = EHCI_LIST_END(ft313);
	INIT_LIST_HEAD (&qtd->qtd_list);

	DEBUG_MSG("Init qTD content\n");
	ft313_mem_write(ft313, qtd, sizeof(struct ehci_qtd_hw), qtd->qtd_ft313);
}

static struct ehci_qtd *ft313_qtd_alloc (struct ft313_hcd *ft313, gfp_t flags)
{
	struct ehci_qtd		*qtd;
	dma_addr_t		dma;
	struct ft313_mem_blk	*mem_blk_ptr = NULL;

	mem_blk_ptr = allocate_mem_blk(ft313, QTD, 0);
	if (mem_blk_ptr == NULL)
		return NULL;
#ifdef DMA_DEVICE
	qtd = dma_pool_alloc (ft313->qtd_pool, flags, &dma);
#else
	qtd = kmalloc(sizeof(struct ehci_qtd), GFP_DMA | GFP_ATOMIC);
	dma = virt_to_phys(qtd);
#endif
	if (qtd != NULL) {
		qtd->qtd_ft313 = mem_blk_ptr->offset;
		ft313_qtd_init(ft313, qtd, dma);
		qtd->mem_flags = flags;
	}
	else {
		if (NULL != mem_blk_ptr)
			mem_blk_ptr->in_use = FALSE;
	}

	return qtd;
}

static inline void ft313_qtd_free (struct ft313_hcd *ft313, struct ehci_qtd *qtd)
{
	FUN_ENTRY();

	if (qtd->buffer_ft313 != 0) {
		if (0 > free_mem_blk(ft313, qtd->buffer_ft313)) {
			DEBUG_MSG("Cannot find memory block for data buffer 0x%X\n", qtd->buffer_ft313);
		}
		else {
			DEBUG_MSG("Data buffer at 0x%X of qTD 0x%X is freed\n", qtd->buffer_ft313, qtd->qtd_ft313);
		}
	}

	// Free on-chip memory block
	if (free_mem_blk(ft313, qtd->qtd_ft313) < 0) {
		DEBUG_MSG("Cannot find memory block for this qTD 0x%X\n", qtd->qtd_ft313);
		BUG();
	}
	else {
		DEBUG_MSG("qTD 0x%X is freed\n", qtd->qtd_ft313);
	}
#ifdef DMA_DEVICE
	dma_pool_free (ft313->qtd_pool, qtd, qtd->qtd_dma);
#else
	kfree(qtd);
#endif
	FUN_EXIT();
}


static void qh_destroy(struct ehci_qh *qh)
{
	struct ft313_hcd *ft313 = qh->ft313;

	FUN_ENTRY();

	/* clean qtds first, and know this is not linked */
	if (!list_empty (&qh->qtd_list) || qh->qh_next.ptr) {
		DEBUG_MSG("unused qh not empty!\n");
//		BUG (); //FixMe: this cause system crash when testing full speed iso transfer
	}
	if (qh->dummy)
		ft313_qtd_free (ft313, qh->dummy);

	// Free on-chip memory block
	if (free_mem_blk(ft313, qh->qh_ft313) < 0) {
		BUG();
	}
	DEBUG_MSG("qH at 0x%X is freed\n", qh->qh_ft313);
#ifdef DMA_DEVICE
	dma_pool_free(ft313->qh_pool, qh->hw, qh->qh_dma);
#else
	kfree(qh->hw);
#endif
	kfree(qh);

	FUN_EXIT();
}


static struct ehci_qh *ft313_qh_alloc(struct ft313_hcd *ft313, gfp_t flags)
{
	struct ehci_qh		*qh;
	dma_addr_t		dma;
	struct ft313_mem_blk	*mem_blk_ptr = NULL;

	qh = kzalloc(sizeof *qh, GFP_ATOMIC);
	if (!qh)
		goto done;
#ifdef DMA_DEVICE
	qh->hw = (struct ehci_qh_hw *)
		dma_pool_alloc(ft313->qh_pool, flags, &dma);
#else
       qh->hw = kmalloc(sizeof(struct ehci_qh_hw), GFP_DMA | GFP_ATOMIC);
       dma = virt_to_phys(qh->hw);
#endif
	if (!qh->hw)
		goto fail2;

	mem_blk_ptr = allocate_mem_blk(ft313, QHEAD, 0);

	if (!mem_blk_ptr)
		goto fail1;
	memset(qh->hw, 0, sizeof *qh->hw);
	DEBUG_MSG("Init qH content\n");
	ft313_mem_write(ft313, qh->hw, sizeof(struct ehci_qh_hw), mem_blk_ptr->offset);
	qh->refcount = 1;
	qh->ft313 = ft313;
	qh->qh_dma = dma;
	qh->qh_ft313 = mem_blk_ptr->offset;

	INIT_LIST_HEAD (&qh->qtd_list);
	INIT_LIST_HEAD (&qh->urb_list);

	/* dummy td enables safe urb queuing */
	DEBUG_MSG("Create a dummy qTD for qH at 0x%04X\n", qh->qh_ft313);
	qh->dummy = ft313_qtd_alloc (ft313, flags);
	if (qh->dummy == NULL) {
		goto fail;
	}
done:
	return qh;

fail:
	if (mem_blk_ptr)
		mem_blk_ptr->in_use = FALSE;
fail1:
#ifdef DMA_DEVICE
	dma_pool_free(ft313->qh_pool, qh->hw, dma);
#else
	kfree(qh->hw);
#endif

fail2:
	kfree(qh);

	return NULL;
}

/* to share a qh (cpu threads, or hc) */
static inline struct ehci_qh *qh_get (struct ehci_qh *qh)
{
	WARN_ON(!qh->refcount);
	qh->refcount++;
	DEBUG_MSG("qH 0x%X refcount increased to %d\n", qh->qh_ft313, qh->refcount);
	return qh;
}

static inline void qh_put (struct ehci_qh *qh)
{
	if (!--qh->refcount) {
		qh_destroy(qh);
		DEBUG_MSG("qh 0x%X refcount is %d\n and should be removed!", qh->qh_ft313, qh->refcount);
	}
	else {
		DEBUG_MSG("qH 0x%X refcount is %d\n", qh->qh_ft313, qh->refcount);
	}
}


/*-------------------------------------------------------------------------*/

/* The queue heads and transfer descriptors are managed from pools tied
 * to each of the "per device" structures.
 * This is the initialisation and cleanup code.
 */

static void ft313_mem_cleanup (struct ft313_hcd *ft313)
{
	struct ft313_mem_blk *current_blk;
	int i;

	free_cached_lists(ft313); //FixMe: for isochronous support!

	if (ft313->async) {
		if (NULL != ft313->async->qh_next.qh) {
			ERROR_MSG("There is still qH 0x%X pending\n", ft313->async->qh_next.qh->qh_ft313);
		}
		qh_put (ft313->async);
	}
	ft313->async = NULL;

	/* DMA consistent memory and pools */
#ifdef DMA_DEVICE
	if (ft313->qtd_pool)
		dma_pool_destroy (ft313->qtd_pool);
	ft313->qtd_pool = NULL;

	if (ft313->qh_pool) {
		dma_pool_destroy (ft313->qh_pool);
		ft313->qh_pool = NULL;
	}

	if (ft313->itd_pool)
		dma_pool_destroy (ft313->itd_pool);
	ft313->itd_pool = NULL;

	if (ft313->sitd_pool)
		dma_pool_destroy (ft313->sitd_pool);
	ft313->sitd_pool = NULL;
#endif

	if (ft313->periodic)
#ifdef DMA_DEVICE
		dma_free_coherent (ft313_to_hcd(ft313)->self.controller,
			ft313->periodic_size * sizeof (u32),
			ft313->periodic, ft313->periodic_dma);
#else
		kfree(ft313->periodic);
#endif
	ft313->periodic = NULL;

	/* shadow periodic table */
	kfree(ft313->pshadow);
	ft313->pshadow = NULL;

	current_blk = ft313->mem;
	for (i = 0; i < ft313->mem_blk_num; i++) {
		current_blk->in_use = FALSE;
		current_blk++;
	}
	kfree(ft313->mem);
}


int ft313_mem_init(struct ft313_hcd *ft313, gfp_t flags)
{
	int i;

	/* On chip memory initlization */
	if (ft313_mem_blk_init(ft313, flags) < 0)
		return -ENOMEM;

	spin_lock_init(&ft313->mem_lock);
	spin_lock_init(&ft313->dataport_lock);

#ifdef DMA_DEVICE
	/* QTDs for control/bulk/intr transfers */
	ft313->qtd_pool = dma_pool_create ("ft313_qtd",
			ft313_to_hcd(ft313)->self.controller,
			sizeof (struct ehci_qtd),
			32 /* byte alignment (for hw parts) */,
			4096 /* can't cross 4K */);
	if (!ft313->qtd_pool) {
		goto fail;
	}

	/* QHs for control/bulk/intr transfers */
	ft313->qh_pool = dma_pool_create ("ft313_qh",
			ft313_to_hcd(ft313)->self.controller,
			sizeof(struct ehci_qh_hw),
			32 /* byte alignment (for hw parts) */,
			4096 /* can't cross 4K */);
	if (!ft313->qh_pool) {
		goto fail;
	}
#endif

	ft313->async = ft313_qh_alloc(ft313, flags);
	if (!ft313->async) {
		goto fail;
	}
#ifdef DMA_DEVICE
	/* ITD for high speed ISO transfers */
	ft313->itd_pool = dma_pool_create ("ft313_itd",
			ft313_to_hcd(ft313)->self.controller,
			sizeof (struct ehci_itd),
			32 /* byte alignment (for hw parts) */,
			4096 /* can't cross 4K */);

	if (!ft313->itd_pool) {
		goto fail;
	}
	ALERT_MSG("Create DMA Pool for iTD end\n");

	/* SITD for full/low speed split ISO transfers */
	ft313->sitd_pool = dma_pool_create ("ft313_sitd",
			ft313_to_hcd(ft313)->self.controller,
			sizeof (struct ehci_sitd),
			32 /* byte alignment (for hw parts) */,
			4096 /* can't cross 4K */);
	if (!ft313->sitd_pool) {
		goto fail;
	}
#endif
	/* Hardware periodic table */
#ifdef DMA_DEVICE
	ft313->periodic = (__le32 *)
		dma_alloc_coherent (ft313_to_hcd(ft313)->self.controller,
			ft313->periodic_size * sizeof(__le32),
			&ft313->periodic_dma, 0);
#else
	ft313->periodic = (__le32 *)kmalloc(ft313->periodic_size * sizeof(__le32), GFP_DMA);
	ft313->periodic_dma = virt_to_phys(ft313->periodic);
#endif
	if (ft313->periodic == NULL) {
		goto fail;
	}

	for (i = 0; i < ft313->periodic_size; i++)
		ft313->periodic[i] = EHCI_LIST_END(ft313);
	// The offset of periodic list base is set as ZERO
	ft313->periodic_ft313 = 0;
	ft313_mem_write(ft313, ft313->periodic, ft313->periodic_size * sizeof(__le32), ft313->periodic_ft313);

	/* software shadow of hardware table */
	ft313->pshadow = kcalloc(ft313->periodic_size, sizeof(void *), flags);
	if (ft313->pshadow != NULL)
		return 0;

fail:
	ALERT_MSG ("couldn't init memory\n");
	ft313_mem_cleanup (ft313);
	return -ENOMEM;
}

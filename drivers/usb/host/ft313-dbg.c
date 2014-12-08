/*
 * FT313 debug support.
 *
 * Copyright (C) 2011 Chang Yang <chang.yang@ftdichip.com>
 *
 * This code is *strongly* based on EHCI-HCD code by David Brownell since
 * the chip is a quasi-EHCI compatible.
 *
 * Licensed under GPL version 2 only.
 */

/* this file is part of ft313-hcd.c */

static inline u32 ft313_reg_read32(const struct ft313_hcd *ft313, void __iomem * regs);
static inline u32 _ft313_reg_read32(const struct ft313_hcd *ft313, void __iomem * regs);
static inline u16 _ft313_reg_read16(const struct ft313_hcd *ft313, void __iomem * regs);

#define ft313_dbg(ft313, fmt, args...) \
dev_dbg (ft313_to_hcd(ft313)->self.controller , fmt , ## args )
#define ft313_err(ft313, fmt, args...) \
dev_err (ft313_to_hcd(ft313)->self.controller , fmt , ## args )
#define ft313_info(ft313, fmt, args...) \
dev_info (ft313_to_hcd(ft313)->self.controller , fmt , ## args )
#define ft313_warn(ft313, fmt, args...) \
dev_warn (ft313_to_hcd(ft313)->self.controller , fmt , ## args )

#ifdef	DEBUG

static void __maybe_unused
dbg_qtd (const char *label, struct ehci_hcd *ehci, struct ehci_qtd *qtd)
{
	ehci_dbg(ehci, "%s td %p n%08x %08x t%08x p0=%08x\n", label, qtd,
		hc32_to_cpup(ehci, &qtd->hw_next),
		hc32_to_cpup(ehci, &qtd->hw_alt_next),
		hc32_to_cpup(ehci, &qtd->hw_token),
		hc32_to_cpup(ehci, &qtd->hw_buf [0]));
	if (qtd->hw_buf [1])
		ehci_dbg(ehci, "  p1=%08x p2=%08x p3=%08x p4=%08x\n",
			hc32_to_cpup(ehci, &qtd->hw_buf[1]),
			hc32_to_cpup(ehci, &qtd->hw_buf[2]),
			hc32_to_cpup(ehci, &qtd->hw_buf[3]),
			hc32_to_cpup(ehci, &qtd->hw_buf[4]));
}

static void __maybe_unused
dbg_qh (const char *label, struct ehci_hcd *ehci, struct ehci_qh *qh)
{
	struct ehci_qh_hw *hw = qh->hw;

	ehci_dbg (ehci, "%s qh %p n%08x info %x %x qtd %x\n", label,
		qh, hw->hw_next, hw->hw_info1, hw->hw_info2, hw->hw_current);
	dbg_qtd("overlay", ehci, (struct ehci_qtd *) &hw->hw_qtd_next);
}

static void __maybe_unused
dbg_itd (const char *label, struct ehci_hcd *ehci, struct ehci_itd *itd)
{
	ehci_dbg (ehci, "%s [%d] itd %p, next %08x, urb %p\n",
		label, itd->frame, itd, hc32_to_cpu(ehci, itd->hw_next),
		itd->urb);
	ehci_dbg (ehci,
		"  trans: %08x %08x %08x %08x %08x %08x %08x %08x\n",
		hc32_to_cpu(ehci, itd->hw_transaction[0]),
		hc32_to_cpu(ehci, itd->hw_transaction[1]),
		hc32_to_cpu(ehci, itd->hw_transaction[2]),
		hc32_to_cpu(ehci, itd->hw_transaction[3]),
		hc32_to_cpu(ehci, itd->hw_transaction[4]),
		hc32_to_cpu(ehci, itd->hw_transaction[5]),
		hc32_to_cpu(ehci, itd->hw_transaction[6]),
		hc32_to_cpu(ehci, itd->hw_transaction[7]));
	ehci_dbg (ehci,
		"  buf:   %08x %08x %08x %08x %08x %08x %08x\n",
		hc32_to_cpu(ehci, itd->hw_bufp[0]),
		hc32_to_cpu(ehci, itd->hw_bufp[1]),
		hc32_to_cpu(ehci, itd->hw_bufp[2]),
		hc32_to_cpu(ehci, itd->hw_bufp[3]),
		hc32_to_cpu(ehci, itd->hw_bufp[4]),
		hc32_to_cpu(ehci, itd->hw_bufp[5]),
		hc32_to_cpu(ehci, itd->hw_bufp[6]));
	ehci_dbg (ehci, "  index: %d %d %d %d %d %d %d %d\n",
		itd->index[0], itd->index[1], itd->index[2],
		itd->index[3], itd->index[4], itd->index[5],
		itd->index[6], itd->index[7]);
}

static void __maybe_unused
dbg_sitd (const char *label, struct ehci_hcd *ehci, struct ehci_sitd *sitd)
{
	ehci_dbg (ehci, "%s [%d] sitd %p, next %08x, urb %p\n",
		label, sitd->frame, sitd, hc32_to_cpu(ehci, sitd->hw_next),
		sitd->urb);
	ehci_dbg (ehci,
		"  addr %08x sched %04x result %08x buf %08x %08x\n",
		hc32_to_cpu(ehci, sitd->hw_fullspeed_ep),
		hc32_to_cpu(ehci, sitd->hw_uframe),
		hc32_to_cpu(ehci, sitd->hw_results),
		hc32_to_cpu(ehci, sitd->hw_buf[0]),
		hc32_to_cpu(ehci, sitd->hw_buf[1]));
}

static int __maybe_unused
dbg_status_buf (char *buf, unsigned len, const char *label, u32 status)
{
	return scnprintf (buf, len,
		"%s%sstatus %04x%s%s%s%s%s%s%s%s%s%s%s",
		label, label [0] ? " " : "", status,
		(status & STS_PPCE_MASK) ? " PPCE" : "",
		(status & STS_ASS) ? " Async" : "",
		(status & STS_PSS) ? " Periodic" : "",
		(status & STS_RECL) ? " Recl" : "",
		(status & STS_HALT) ? " Halt" : "",
		(status & STS_IAA) ? " IAA" : "",
		(status & STS_FATAL) ? " FATAL" : "",
		(status & STS_FLR) ? " FLR" : "",
		(status & STS_PCD) ? " PCD" : "",
		(status & STS_ERR) ? " ERR" : "",
		(status & STS_INT) ? " INT" : ""
		);
}

static int __maybe_unused
dbg_intr_buf (char *buf, unsigned len, const char *label, u32 enable)
{
	return scnprintf (buf, len,
		"%s%sintrenable %02x%s%s%s%s%s%s%s",
		label, label [0] ? " " : "", enable,
		(enable & STS_PPCE_MASK) ? " PPCE" : "",
		(enable & STS_IAA) ? " IAA" : "",
		(enable & STS_FATAL) ? " FATAL" : "",
		(enable & STS_FLR) ? " FLR" : "",
		(enable & STS_PCD) ? " PCD" : "",
		(enable & STS_ERR) ? " ERR" : "",
		(enable & STS_INT) ? " INT" : ""
		);
}

static const char *const fls_strings [] =
    { "1024", "512", "256", "??" };

static int
dbg_command_buf (char *buf, unsigned len, const char *label, u32 command)
{
	return scnprintf (buf, len,
		"%s%scommand %07x %s%s%s%s%s%s=%d ithresh=%d%s%s%s%s "
		"period=%s%s %s",
		label, label [0] ? " " : "", command,
		(command & CMD_HIRD) ? " HIRD" : "",
		(command & CMD_PPCEE) ? " PPCEE" : "",
		(command & CMD_FSP) ? " FSP" : "",
		(command & CMD_ASPE) ? " ASPE" : "",
		(command & CMD_PSPE) ? " PSPE" : "",
		(command & CMD_PARK) ? " park" : "(park)",
		CMD_PARK_CNT (command),
		(command >> 16) & 0x3f,
		(command & CMD_LRESET) ? " LReset" : "",
		(command & CMD_IAAD) ? " IAAD" : "",
		(command & CMD_ASE) ? " Async" : "",
		(command & CMD_PSE) ? " Periodic" : "",
		fls_strings [(command >> 2) & 0x3],
		(command & CMD_RESET) ? " Reset" : "",
		(command & CMD_RUN) ? "RUN" : "HALT"
		);
}

static int
dbg_port_buf (char *buf, unsigned len, const char *label, int port, u32 status)
{
	char	*sig;

	/* signaling state */
	switch (status & (3 << 10)) {
	case 0 << 10: sig = "se0"; break;
	case 1 << 10: sig = "k"; break;		/* low speed */
	case 2 << 10: sig = "j"; break;
	default: sig = "?"; break;
	}

	return scnprintf (buf, len,
		"%s%sport:%d status %06x %d %s%s%s%s%s%s "
		"sig=%s%s%s%s%s%s%s%s%s%s%s",
		label, label [0] ? " " : "", port, status,
		status>>25,/*device address */
		(status & PORT_SSTS)>>23 == PORTSC_SUSPEND_STS_ACK ?
						" ACK" : "",
		(status & PORT_SSTS)>>23 == PORTSC_SUSPEND_STS_NYET ?
						" NYET" : "",
		(status & PORT_SSTS)>>23 == PORTSC_SUSPEND_STS_STALL ?
						" STALL" : "",
		(status & PORT_SSTS)>>23 == PORTSC_SUSPEND_STS_ERR ?
						" ERR" : "",
		(status & PORT_POWER) ? " POWER" : "",
		(status & PORT_OWNER) ? " OWNER" : "",
		sig,
		(status & PORT_LPM) ? " LPM" : "",
		(status & PORT_RESET) ? " RESET" : "",
		(status & PORT_SUSPEND) ? " SUSPEND" : "",
		(status & PORT_RESUME) ? " RESUME" : "",
		(status & PORT_OCC) ? " OCC" : "",
		(status & PORT_OC) ? " OC" : "",
		(status & PORT_PEC) ? " PEC" : "",
		(status & PORT_PE) ? " PE" : "",
		(status & PORT_CSC) ? " CSC" : "",
		(status & PORT_CONNECT) ? " CONNECT" : "");
}

#else
static inline void __maybe_unused
dbg_qh (char *label, struct ft313_hcd *ft313, struct ehci_qh *qh)
{}

static inline int __maybe_unused
dbg_status_buf (char *buf, unsigned len, const char *label, u32 status)
{ return 0; }

static inline int __maybe_unused
dbg_command_buf (char *buf, unsigned len, const char *label, u32 command)
{ return 0; }

static inline int __maybe_unused
dbg_intr_buf (char *buf, unsigned len, const char *label, u32 enable)
{ return 0; }

static inline int __maybe_unused
dbg_port_buf (char *buf, unsigned len, const char *label, int port, u32 status)
{ return 0; }

#endif	/* DEBUG */


/* functions have the "wrong" filename when they're output... */
#define dbg_status(ft313, label, status) { \
	char _buf [80]; \
	dbg_status_buf (_buf, sizeof _buf, label, status); \
	ft313_dbg (ft313, "%s\n", _buf); \
}

#define dbg_cmd(ft313, label, command) { \
	char _buf [80]; \
	dbg_command_buf (_buf, sizeof _buf, label, command); \
	ft313_dbg (ft313, "%s\n", _buf); \
}

#define dbg_port(ft313, label, port, status) { \
	char _buf [80]; \
	dbg_port_buf (_buf, sizeof _buf, label, port, status); \
	ft313_dbg (ft313, "%s\n", _buf); \
}


/*
	ft313 chip access log function
*/
enum reg_index {
	HC_CAP_REG = 0,
	HCSPARAMS,
	HCCPARAMS,
	USBCMD,
	USBSTS,
	USBINTR,
	FRINDEX,
	PERIODICLISTADDR,
	ASYNCLISTADDR,
	PORTSC,

	EOFTIME,
//	BUS_MON_CTRL_STATUS,
//	BUS_MON_INT_STATUS,
//	BUS_MON_INT_ENABLE,
	TESTMODE,
	TESTPMSET1,
	TESTPMSET2,
	CHIPID,
	HWMODE,
	EDGEINTC,
	SWRESET,
	MEMADDR,
	DATAPORT,
	DATASESSION,
	CONFIG,
	AUX_MEMADDR,
	AUX_DATAPORT,
	SLEEPTIMER,
	HCINTSTS,
	HCINTEN,

	END_OF_REG,
};

struct reg_info {
	enum reg_index	reg_idx;
	u32		reg_offset;
	u32		num_bits;
	char		friendly_name[16];
};

struct reg_info ft313_reg_map[] = {
	{HC_CAP_REG,		0x00,	32,	"HCCAP      "},
	{HCSPARAMS,		0x04,	32,	"HCSPARAMS  "},
	{HCCPARAMS,		0x08,	32,	"HCCPARAMS  "},
	{USBCMD,		0x10,	32,	"USBCMD     "},
	{USBSTS,		0x14,	32,	"USBSTS     "},
	{USBINTR,		0x18,	32,	"USBINTR    "},
	{FRINDEX,		0x1C, 	32,	"FRINDEX    "},
	{PERIODICLISTADDR,	0x24,	32,	"PERIODADDR "},
	{ASYNCLISTADDR,		0x28,	32,	"ASYNCADDR  "},
	{PORTSC,		0x30,	32,	"PORTSC     "},
	{EOFTIME,		0x34,	32,	"EOFTIME    "},
	{TESTMODE,		0x50,	32,	"TESTMODE   "},
	{TESTPMSET1,		0x70,	32,	"TESTPMSET1 "},
	{TESTPMSET2,		0x74,	32,	"TESTPMSET2 "},
	{CHIPID,		0x80,	32,	"CHIPID     "},
	{HWMODE,		0x84,	16,	"HWMODE     "},
	{EDGEINTC,		0x88,	32,	"EDGEINTC   "},
	{SWRESET,		0x8C,	16,	"SWRESET    "},
	{MEMADDR,		0x90,	16,	"MEMADDR    "},
	{DATAPORT,		0x92,	16,	"DATAPORT   "},
	{DATASESSION,		0x94,	16,	"DATASESS   "},
	{CONFIG,		0x96,	16,	"CONFIG     "},
	{AUX_MEMADDR,		0x98,	16,	"AUXMEMADDR "},
	{AUX_DATAPORT,		0x9A,	16,	"AUXDATAPORT"},
	{SLEEPTIMER,		0x9C,	16,	"SLEEPTIMER "},
	{HCINTSTS,		0xA0,	16,	"HCINTSTS   "},
	{HCINTEN,		0xA4,	16,	"HCINTEN    "},

	{END_OF_REG,		0xFF,	32,	"EOR"},
};

#define IO_READ		0x01
#define IO_WRITE	0x00

#define SRAM_MEM_OFFSET0 	0x200000

void *g_regbase = NULL;
u32  g_print_cnt = 0;

#ifdef DEBUG_MSG_ON

#define DEBUG_MSG(fmt, args...) do { \
printk(KERN_ERR "%s, line=%d, %s: ",  __FILE__, __LINE__, __func__); \
printk(KERN_ERR fmt, ##args); \
} while(0)

#define ERROR_MSG(fmt, args...) do { \
printk(KERN_ERR "FT313 Error at %s, line=%d, %s: ",  __FILE__, __LINE__, __func__); \
printk(KERN_ERR fmt, ##args); \
} while(0)

#define FUN_ENTRY() do { \
printk("\n%s: Enter++ at line %d\n", __func__, __LINE__); \
} while(0)

#define FUN_EXIT() do { \
	printk("%s: Exit-- at line %d\n\n", __func__, __LINE__); \
} while(0)

#else

#define DEBUG_MSG(fmt, args...)

#define ERROR_MSG(fmt, args...)

#define FUN_ENTRY()

#define FUN_EXIT()

#endif // DEBUG_MSG_ON

#define ALERT_MSG(fmt, args...) do { \
printk(KERN_ALERT "FT313 Alert at %s, line=%d, %s: ",  __FILE__, __LINE__, __func__); \
printk(KERN_ALERT fmt, ##args); \
} while(0)

struct reg_info *get_reg_info(int offset)
{
	struct reg_info *reg;

	reg = &ft313_reg_map[0];

	if (offset < 0)
		return NULL;

	while (reg->reg_idx != END_OF_REG) {
		if (reg->reg_offset == offset) {
			return reg;
		}
		reg++;
	}

	return NULL;
}

void print_mem_access_info(int access_type, u32 mem_offset, u32 length, void* buf)
{
//	printk("No. %d op\t: ", ++g_print_cnt);

	if ((mem_offset + length) >= FT313_CHIP_MEM_AMT) {
		printk("Memory access beyond memory region!\n");
		return;
	}

	if (access_type == IO_READ) {
		printk("Read %d bytes long memory segment from offset 0x%08X to system mem 0x%p\n", length, mem_offset, buf);
	}
	else if (access_type == IO_WRITE) {
		printk("Write %d bytes long memory segment from offset 0x%08X from system mem 0x%p\n", length, mem_offset, buf);
	}
	else {
		printk("Wrong access type \n");
	}
}

void print_regs(struct ft313_hcd *ft313)
{
	struct reg_info *reg;
	struct usb_hcd * hcd = ft313_to_hcd(ft313);

	reg = &ft313_reg_map[0];

	while (reg->reg_idx != END_OF_REG) {
		if (reg->num_bits == 32)
			printk(KERN_NOTICE "%s: 0x%04X -> 0x%08X\n", 
				reg->friendly_name, reg->reg_offset,
				_ft313_reg_read32(ft313, (void *)hcd->regs + reg->reg_offset));
		else
			printk(KERN_NOTICE "%s: 0x%04X -> 0x%04X\n", 
				reg->friendly_name, reg->reg_offset,
				_ft313_reg_read16(ft313, (void *)hcd->regs + reg->reg_offset));
		
		reg++;
	}	
}


void print_reg_access_info(int access_type, void* reg, u32 value, u32 num_bits)
{
	int offset;
	struct reg_info *reginfo = NULL;


	if (reg < g_regbase) {
		offset = -1;
	} else {
		offset = reg - g_regbase;
		reginfo = get_reg_info(offset);
		if(reginfo->num_bits != num_bits)
			printk(KERN_ERR "************************** ERROR INVALID REG ACCESS **************\n");
	}

//	printk("No. %d op\t: ", ++g_print_cnt);

	if (access_type == IO_READ) {
		if (num_bits == 32)
			printk(KERN_NOTICE "FT313 RD32: %s: 0x%04X -> 0x%08X\n", 
				reginfo ? reginfo->friendly_name : "UNKNOWN", offset, value);
		else
			printk(KERN_NOTICE "FT313 RD16: %s: 0x%04X -> 0x%04X\n", 
				reginfo ? reginfo->friendly_name : "UNKNOWN", offset, value);

	} else if (access_type == IO_WRITE) {
		if (num_bits == 32)
			printk(KERN_NOTICE "FT313 WR32: %s: 0x%04X <- 0x%08X\n", 
				reginfo ? reginfo->friendly_name : "UNKNOWN", offset, value);
		else
			printk(KERN_NOTICE "FT313 WR16: %s: 0x%04X <- 0x%04X\n", 
				reginfo ? reginfo->friendly_name : "UNKNOWN", offset, value);
	} else {
		printk("Wrong access type \n");
	}
}


static u8 tmpbuf[4096];
void display_period_list(struct ft313_hcd *ft313)
{
	int i;

	printk(KERN_NOTICE "PERIODIC LIST (%d elements)", ft313->periodic_size);
	ft313_mem_read(ft313, tmpbuf, 4096, 0);
	for (i = 0; i < 4096; i += 4) {
		printk(KERN_NOTICE "%04x %08X\n", i, *(u32*)(tmpbuf + i));
	}
}


#if 0
void display_async_list(struct ft313_hcd* ft313)
{
	struct ehci_qh *prev;
	u32 horizontal_ptr;

	printk("\n\nFT313 Async List as:\n\n");
	prev = ft313->async;
	ft313_mem_read(ft313, &horizontal_ptr, 4, prev->qh_ft313 + offsetof(struct ehci_qh_hw, hw_next));
	printk("From qh 0x%X (horizon ptr 0x%X) -> ", prev->qh_ft313, horizontal_ptr);

	while (prev->qh_next.qh != NULL) {
		prev = prev->qh_next.qh;
		ft313_mem_read(ft313, &horizontal_ptr, 4, prev->qh_ft313 + offsetof(struct ehci_qh_hw, hw_next));
		printk("qh 0x%X (horizon ptr 0x%X)-> ", prev->qh_ft313, horizontal_ptr);
	}
	printk(" NULL \n");

	u32 async_reg;

	async_reg = ft313_reg_read32(ft313, &ft313->regs->async_next);

	printk("Current Async List Address at 0x%X\n\n\n", async_reg);
}
#endif

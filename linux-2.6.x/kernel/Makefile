#
# Makefile for the linux kernel.
#

obj-y     = sched.o fork.o exec_domain.o panic.o printk.o profile.o \
	    exit.o itimer.o time.o softirq.o resource.o \
	    sysctl.o capability.o ptrace.o timer.o user.o \
	    signal.o sys.o kmod.o workqueue.o pid.o \
	    rcupdate.o intermodule.o extable.o params.o posix-timers.o \
	    kthread.o wait.o kfifo.o sys_ni.o lspinfo.o

obj-$(CONFIG_PREEMPT_RT) += rt.o

obj-$(CONFIG_DEBUG_PREEMPT) += latency.o
obj-$(CONFIG_LATENCY_TIMING) += latency.o

obj-$(CONFIG_FUTEX) += futex.o
obj-$(CONFIG_HIGH_RES_TIMERS) += hrtimers.o
obj-$(CONFIG_GENERIC_ISA_DMA) += dma.o
obj-$(CONFIG_SMP) += cpu.o spinlock.o
obj-$(CONFIG_UID16) += uid16.o
ifeq ($(CONFIG_VST),y)
	DOVST=vst.o
endif
ifeq ($(CONFIG_IDLE),y)
	DOVST=vst.o
endif
obj-y += $(DOVST)
obj-$(CONFIG_MODULES) += module.o
obj-$(CONFIG_KALLSYMS) += kallsyms.o
obj-$(CONFIG_PM) += power/
obj-$(CONFIG_BSD_PROCESS_ACCT) += acct.o
obj-$(CONFIG_LTT) += ltt-core.o
obj-$(CONFIG_COMPAT) += compat.o
obj-$(CONFIG_IKCONFIG) += configs.o
obj-$(CONFIG_IKCONFIG_PROC) += configs.o
obj-$(CONFIG_STOP_MACHINE) += stop_machine.o
obj-$(CONFIG_AUDIT) += audit.o
obj-$(CONFIG_AUDITSYSCALL) += auditsc.o
obj-$(CONFIG_GPIO) += gpio.o
obj-$(CONFIG_KPROBES) += kprobes.o
obj-$(CONFIG_KGDB) += kgdb.o
obj-$(CONFIG_SYSFS) += ksysfs.o
obj-$(CONFIG_GENERIC_HARDIRQS) += irq/

ifneq ($(CONFIG_IA64),y)
# According to Alan Modra <alan@linuxcare.com.au>, the -fno-omit-frame-pointer is
# needed for x86 only.  Why this used to be enabled for all architectures is beyond
# me.  I suspect most platforms don't need this, but until we know that for sure
# I turn this off for IA-64 only.  Andreas Schwab says it's also needed on m68k
# to get a correct value for the wait-channel (WCHAN in ps). --davidm
CFLAGS_sched.o := $(PROFILING) -fno-omit-frame-pointer
endif

$(obj)/configs.o: $(obj)/config_data.h

# config_data.h contains the same information as ikconfig.h but gzipped.
# Info from config_data can be extracted from /proc/config*
targets += config_data.gz
$(obj)/config_data.gz: .config FORCE
	$(call if_changed,gzip)

quiet_cmd_ikconfiggz = IKCFG   $@
      cmd_ikconfiggz = (echo "const char kernel_config_data[] = MAGIC_START"; cat $< | scripts/bin2c; echo "MAGIC_END;") > $@
targets += config_data.h
$(obj)/config_data.h: $(obj)/config_data.gz FORCE
	$(call if_changed,ikconfiggz)

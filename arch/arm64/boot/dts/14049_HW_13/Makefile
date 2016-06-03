ifeq ($(CONFIG_OF),y)
dtb-$(CONFIG_ARCH_MSM8994) += msm8994-v1-mtp_14049_HW_13.dtb \
	msm8994-v2.0-mtp_14049_HW_13.dtb \
	msm8994-v2.1-mtp_14049_HW_13.dtb

DTB_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM_APPENDED_DTB_IMAGE_NAMES))
ifneq ($(DTB_NAMES),)
DTB_LIST := $(addsuffix .dtb,$(DTB_NAMES))
else
DTB_LIST := $(dtb-y)
endif

targets += dtbs
targets += $(addprefix ../, $(DTB_LIST))
endif

$(obj)/../%.dtb: $(src)/%.dts FORCE
	$(call if_changed_dep,dtc)

dtbs: $(addprefix $(obj)/../,$(DTB_LIST))

clean-files := *.dtb

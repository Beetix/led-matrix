# Component makefile for ugui

INC_DIRS += $(ugui_ROOT)

# args for passing into compile rule generation
ugui_SRC_DIR =  $(ugui_ROOT)

$(eval $(call component_compile_rules,ugui))

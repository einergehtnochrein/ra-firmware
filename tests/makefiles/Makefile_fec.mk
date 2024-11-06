COMPONENT_NAME=fec

SRC_FILES = \
  $(PROJECT_SRC_DIR)/src/fec/reedsolomon/reedsolomon.c \
  $(PROJECT_SRC_DIR)/src_m0/viterbi.c

TEST_SRC_FILES = \
  $(UNITTEST_SRC_DIR)/test_fec.cpp

UNITTEST_EXTRA_INC_PATHS += \
  -I$(UNITTEST_ROOT)/stubs \
  -I$(PROJECT_SRC_DIR) \
  -I$(PROJECT_SRC_DIR)/src

include $(CPPUTEST_MAKFILE_INFRA)



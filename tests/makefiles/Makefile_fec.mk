COMPONENT_NAME=fec

SRC_FILES = \
  $(PROJECT_SRC_DIR)/src/reedsolomon/reedsolomon.c

TEST_SRC_FILES = \
  $(UNITTEST_SRC_DIR)/test_fec.cpp

UNITTEST_EXTRA_INC_PATHS += \
  -I$(PROJECT_SRC_DIR)/src

include $(CPPUTEST_MAKFILE_INFRA)



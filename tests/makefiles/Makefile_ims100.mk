COMPONENT_NAME=ims100

SRC_FILES = \
  $(PROJECT_SRC_DIR)/src/fec/bch/bch.c \
  $(PROJECT_SRC_DIR)/src/meisei/meiseimetrology.c \
  $(PROJECT_SRC_DIR)/src/meisei/meiseiutils.c \
  $(UNITTEST_SRC_DIR)/../fakes/fake_meiseicalibconfig.c \

TEST_SRC_FILES = \
  $(UNITTEST_ROOT)/stubs/thumb.c \
  $(UNITTEST_SRC_DIR)/test_ims100.cpp

UNITTEST_EXTRA_INC_PATHS += \
  -I$(UNITTEST_ROOT)/stubs \
  -I$(PROJECT_SRC_DIR)/src \
  -I$(PROJECT_SRC_DIR)/src/fec/bch

include $(CPPUTEST_MAKFILE_INFRA)



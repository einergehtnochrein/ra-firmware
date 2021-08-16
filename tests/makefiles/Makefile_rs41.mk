COMPONENT_NAME=rs41

SRC_FILES = \
  $(PROJECT_SRC_DIR)/src/rs41/rs41metrology.c \
  $(UNITTEST_SRC_DIR)/../fakes/fake_rs41calibconfig.c

TEST_SRC_FILES = \
  $(UNITTEST_SRC_DIR)/test_rs41.cpp

include $(CPPUTEST_MAKFILE_INFRA)



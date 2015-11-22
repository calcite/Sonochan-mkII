BUILD_NUM_A=`cat build_num_A.txt`
BUILD_NUM_B=`cat build_num_B.txt`
BUILD_NUM_C=`cat build_num_C.txt`

# We need to remove all .o files where build number is used
build_cnt:
	@rm src/display.o
	@echo "Build version ["$(BUILD_NUM_A)"."$(BUILD_NUM_B)"."$(BUILD_NUM_C)"]"
	@if [ $(BUILD_NUM_C) -ge 99 ] ; then \
	   echo "0" > build_num_C.txt ; \
	   if [ $(BUILD_NUM_B) -ge 99 ] ; then \
	     echo "0" > build_num_B.txt ; \
	     echo $$(($(BUILD_NUM_A) + 1 )) > build_num_A.txt ; \
	   else \
	     echo $$(($(BUILD_NUM_B) + 1 )) > build_num_B.txt ; \
	   fi \
	else \
	  echo $$(($(BUILD_NUM_C) + 1 )) > build_num_C.txt ; \
	fi


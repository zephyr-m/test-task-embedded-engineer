.PHONY: test sim ccheck pycheck pytest ctest clean

BUILD_DIR := build
C_TEST_BIN := $(BUILD_DIR)/c_current_loop_tests

test: pycheck ccheck pytest ctest sim

pycheck:
	python3 -m py_compile simulation/pi_controller.py simulation/current_loop_sim.py

ccheck:
	gcc -std=c99 -Wall -Wextra -Werror -Istm32/Inc -c stm32/Src/current_loop_controller.c -o /tmp/current_loop_controller.o

pytest:
	python3 tests/run_python_tests.py

ctest:
	mkdir -p $(BUILD_DIR)
	gcc -std=c99 -Wall -Wextra -Werror -Istm32/Inc \
		stm32/Src/current_loop_controller.c tests/c_current_loop_tests.c \
		-o $(C_TEST_BIN) -lm
	./$(C_TEST_BIN)

sim:
	python3 simulation/current_loop_sim.py

clean:
	rm -rf $(BUILD_DIR) simulation/current_loop_result.csv /tmp/current_loop_controller.o

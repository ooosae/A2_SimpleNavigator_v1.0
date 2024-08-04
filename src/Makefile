CC = g++
CFLAGS = -std=c++17 -Wall -pedantic -Wextra -Inavigator -Inavigator/containers
UNITFLAGS1 := $(shell pkg-config --cflags gtest)
UNITFLAGS2 := $(shell pkg-config --libs gtest)

UNITOBJS = ./out/tests.o
OBJS = ./out/s21_graph.o ./out/s21_graph_algorithms.o
GCOV_FLAGS =

.PHONY: all clean fclean test install uninstall dvi dist gcov_report style_change style_check tests

all: clean install

OS := $(shell uname)
ifeq ($(OS), Darwin)
	TEST_FLAGS = -DGTEST_HAS_PTHREAD=1 -I/usr/local/include -L/usr/local/lib -lgtest -lgtest_main
else
	TEST_FLAGS = -lgtest -lm

endif

install: $(OBJS) s21_graph.a s21_graph_algorithms.a ./out/main.o
	@echo "Start installation"
	@mkdir -p build
	@$(CC) -o ./build/Navigator ./out/main.o $(OBJS) $(CFLAGS)
	@echo "┏=========================================┓"
	@echo "┃    The installation was successful.     ┃"
	@echo "┃     The project is located in the       ┃"
	@echo "┃             build folder.               ┃"
	@echo "┗=========================================┛"
	./build/Navigator


uninstall:
	@rm -rf build/$(Navigator)
	@echo "┏==================================┓"
	@echo "┃ Uninstall completed successfully ┃"
	@echo "┗==================================┛"

dvi:
	@open ../README_RUS.md

dist:
	@mkdir -p dist
	@cp -R build/$(MZ) dist
	@cp ../README_RUS.md dist && cp ../README.md dist
	@cd dist && tar cvzf Maze.tgz *
	@cd dist && rm -rf $(MZ) README.md README_RUS.md
	@echo "The build was successfully archived"

tests : $(OBJS) $(UNITOBJS)
	$(CC) -o tests $(UNITOBJS) $(OBJS) $(TEST_FLAGS) -lstdc++ -fprofile-arcs -ftest-coverage
	./tests

s21_graph.a: ./out/s21_graph.o
	ar rc $@ $^
	ranlib $@
	cp $@ lib$@

s21_graph_algorithms.a: ./out/s21_graph_algorithms.o
	ar rc $@ $^
	ranlib $@
	cp $@ lib$@

./out/%.o : ./%.cpp
	mkdir -p ./out
	$(CC) $(CFLAGS) $(UNITFLAGS1) $(COV_FLAG) -c $<
	mv ./*.o ./out/

./out/%.o : ./navigator/%.cpp
	mkdir -p ./out
	$(CC) $(CFLAGS) $(UNITFLAGS1) $(COV_FLAG) -c $<
	mv ./*.o ./out/

./out/tests.o : ./test/tests.cpp
	mkdir -p ./out
	$(CC) $(CFLAGS) -c $< $(UNITFLAGS1)
	mv ./*.o ./out/

coverage:
	@./test
	@echo "┏==================================┓"
	@echo "┃  Testing completed successfully  ┃"
	@echo "┗==================================┛"

gcov_report: GCOV_FLAGS += --coverage
gcov_report: clean test
	@echo "┏=========================================┓"
	@echo "┃     Generating your html report ...     ┃"
	@echo "┗=========================================┛"
	@lcov -t test -o test.info -c -d . --exclude '*/tests/*' --ignore-errors source
	@genhtml --ignore-errors unmapped -o test_html test.info
	@rm -rf *.gc* test/*.gc*
	@rm -rf *.gd* test/*.gd*
	@open ./test_html/index.html

style_check:
	@echo "┏=========================================┓"
	@echo "┃  Checking your code for Google Style    ┃"
	@echo "┗=========================================┛"
	@cp ../materials/linters/.clang-format .
	@clang-format -n nagivator/*.cpp navigator/*/*.h *.cpp navigator/*.h test/*.cpp
	@rm ./.clang-format

style_change:
	@echo "┏=========================================┓"
	@echo "┃  Checking your code for Google Style    ┃"
	@echo "┗=========================================┛"
	@cp ../materials/linters/.clang-format .
	@clang-format -i nagivator/*.cpp navigator/*/*.h *.cpp navigator/*.h test/*.cpp
	@rm ./.clang-format

clean:
	@rm -rf test/testing *.gcno *.gcov navigator/*.o *.gcda *.info gcov_report
	@rm -rf test/*.gcno test/*.o test/*gcda
	@rm -rf tests test.gcda test.gcno
	@rm -rf build
	@rm -rf *.a
	@rm -rf out


fclean: clean
	@rm -rf dist build* test_html


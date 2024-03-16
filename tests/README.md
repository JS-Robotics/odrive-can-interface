# tests

### Setting up test environment

Install from: https://github.com/google/googletest/tree/main

Guides:
- https://github.com/google/googletest/blob/main/googletest/README.md
- http://google.github.io/googletest/quickstart-cmake.html

Download the repository
git clone https://github.com/google/googletest.git -b v1.14.0

Then install the repository

```shell
cd googletest
mkdir build
cd build
cmake ..
make
sudo make install
```

The above command also includes GoogleMock by default. And so, if you want to build only GoogleTest, you should replace the cmake command with

```shell
cmake .. -DBUILD_GMOCK=OFF
```


Google Test Linking:  `GTest::GTest` and `GTest::Main`. If you're also planning to use Google Mock, `GTest::GTest` and `GTest::Main` target includes both Google Test and Google Mock.
```shell
GTest::GTest
GTest::Main
gmock
gmock_main
```

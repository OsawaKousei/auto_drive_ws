cmake_minimum_required(VERSION 3.8)
# project名
project(model_test)

# コンパイラに引き渡すコンパイルオプションを設定
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
# package.xmlのdependをチェックして自動的にfind_packageしてくれる
ament_auto_find_build_dependencies()

# 実行ファイル「pub_sub_node」をビルド、その際依存しているパッケージに含まれるライブラリ等を全てリンクする
# ament_auto_add_executable(pub_sub_node)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # package.xmlのtest_dependを確認し自動で必要なテストを実行する
  ament_lint_auto_find_test_dependencies()
endif()

# ヘッダーのインストールや共有ライブラリのエクスポート等を自動で行ってくれる
ament_auto_package(
  INSTALL_TO_SHARE
    launch
    models
)
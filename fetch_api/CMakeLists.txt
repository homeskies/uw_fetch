cmake_minimum_required(VERSION 2.8.3)
project(fetch_api)

find_package(catkin REQUIRED COMPONENTS
roslint
)
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
)

# Lint Python modules for PEP8 compatibility
file(GLOB_RECURSE ${PROJECT_NAME}_PY_SRC
        RELATIVE ${PROJECT_SOURCE_DIR} src/${PROJECT_NAME}/*.py)
set(ROSLINT_PYTHON_OPTS "--max-line-length=120")
roslint_python(${${PROJECT_NAME}_PY_SRC})


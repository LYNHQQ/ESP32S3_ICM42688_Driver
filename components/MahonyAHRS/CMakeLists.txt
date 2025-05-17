idf_component_register(
    # 源文件路径需要指向src目录
    SRCS 
        "src/MahonyAHRS.c"
    
    # include目录包含头文件
    INCLUDE_DIRS
        "include"

    # 依赖的其他组件
    REQUIRES
        driver
)
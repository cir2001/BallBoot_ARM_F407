from SCons.Script import Import

Import("env")

# 强制链接参数
env.Append(
    LINKFLAGS=[
        "-mfloat-abi=hard",
        "-mfpu=fpv4-sp-d16",
        "-u", "_printf_float"  # <--- 在这里添加，PlatformIO 就绝对不会搞错了
    ]
)
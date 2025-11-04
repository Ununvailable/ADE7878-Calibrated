Import("env")

env.Replace(
    UPLOADERFLAGS=[
        "-f", "interface/stlink.cfg",
        "-c", "transport select hla_swd",
        "-f", "target/stm32f4x.cfg",
        "-c", "reset_config none",
        "-c", "init",
        "-c", "halt",
        "-c", "program {$SOURCE} verify",
        "-c", "reset run",
        "-c", "exit"
    ]
)
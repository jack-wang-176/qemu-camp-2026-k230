_libstm32f1xx_gpio_rs = static_library(
  'stm32f1xx_gpio',
  files(
    'src/lib.rs',
    'src/gpio.rs',
    'src/bindings.rs',
  ),
  override_options: ['rust_std=2021', 'build.rust_std=2021'],
  rust_abi: 'rust',
  link_with: [
    _util_rs,
    _migration_rs,
    _bql_rs,
    _qom_rs,
    _system_rs,
    _hwcore_rs,
    _trace_rs,
  ],
  dependencies: [
    common_rs,
    glib_sys_rs,
  ],
)

rust_devices_ss.add(when: 'CONFIG_X_STM32F1XX_GPIO_RUST', if_true: [declare_dependency(
  link_whole: [_libstm32f1xx_gpio_rs],
  variables: {'crate': 'stm32f1xx_gpio'},
)])

# Dependency usable by the in-crate integration test below.
stm32f1xx_gpio_rs = declare_dependency(link_with: [_libstm32f1xx_gpio_rs],
  dependencies: [common_rs, qom_rs, hwcore_rs, bql_rs, migration_rs,
                 system_rs, util_rs])

# Native Rust unit tests for the pure register logic (GpioRegisters). These
# need no qtest and no machine boot: they construct GpioRegisters directly and
# only require the BQL mock (bql::start_test). Run with:
#   meson test --suite rust rust-stm32f1xx-gpio-integration
test('rust-stm32f1xx-gpio-integration',
    executable(
        'rust-stm32f1xx-gpio-integration',
        files('tests/tests.rs'),
        override_options: ['rust_std=2021', 'build.rust_std=2021'],
        rust_args: ['--test'],
        install: false,
        dependencies: [stm32f1xx_gpio_rs, common_rs, qom_rs, hwcore_rs,
                       bql_rs, migration_rs, system_rs, util_rs]),
    args: [
        '--test', '--test-threads', '1',
        '--format', 'pretty',
    ],
    protocol: 'rust',
    suite: ['unit', 'rust'])


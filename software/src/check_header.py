Import("env", "projenv")

from pathlib import Path
import shlex
import shutil
import subprocess


PROJECT_DIR = Path(env.subst("$PROJECT_DIR"))
SOURCE_DIR = PROJECT_DIR / "src"
BUILD_DIR = Path(env.subst("$BUILD_DIR"))
GENERATED_DIR = BUILD_DIR / "header_checks"

# Add or remove headers as needed.
HEADERS = [
    "ArduinoCpp17Fix.h",
    "CANMotorMIT.h",
    "Encoder.h",
    "LoadCell.h",
    "MotorConfig.h",
    "ProtocolTypes.h",
    "SerialConfig.h",
    "SerialMotorControl.h",
    "SerialProtocol.h",
    "ServoCANMotor.h",
    "Board.h",
]


def add_include_dir(include_dirs: list[Path], directory) -> None:
    """Add an existing include directory without duplicates."""
    if not directory:
        return

    path = Path(env.subst(str(directory))).resolve()

    if path.is_dir() and path not in include_dirs:
        include_dirs.append(path)


def add_library_tree(include_dirs: list[Path], libraries_dir: Path) -> None:
    """
    Add library roots and src directories under a PlatformIO library folder.

    Examples:
        .pio/libdeps/<environment>/ArduinoJson/src
        framework-arduinoteensy/libraries/SPI
    """
    if not libraries_dir.is_dir():
        return

    for library_dir in libraries_dir.iterdir():
        if not library_dir.is_dir():
            continue

        add_include_dir(include_dirs, library_dir)
        add_include_dir(include_dirs, library_dir / "src")


def get_framework_directories() -> list[Path]:
    """Return framework directories available to the active environment."""
    platform = env.PioPlatform()

    package_names = (
        "framework-arduinoteensy",
        "framework-arduino-mbed",
        "framework-arduinonordicnrf5",
    )

    framework_dirs: list[Path] = []

    for package_name in package_names:
        # A package name may not exist for the active platform.
        if package_name not in platform.packages:
            continue

        try:
            package_dir = platform.get_package_dir(package_name)
        except (KeyError, ValueError):
            continue

        if not package_dir:
            continue

        path = Path(package_dir).resolve()

        if path.is_dir() and path not in framework_dirs:
            framework_dirs.append(path)

    return framework_dirs
def collect_include_directories() -> list[Path]:
    include_dirs = []

    # Project headers.
    add_include_dir(include_dirs, SOURCE_DIR)
    add_include_dir(include_dirs, PROJECT_DIR / "include")
    add_include_dir(include_dirs, PROJECT_DIR / "lib")

    # Include paths already known to the active PlatformIO environment.
    for include_dir in projenv.get("CPPPATH", []):
        add_include_dir(include_dirs, include_dir)

    # Environment-specific project dependencies:
    # .pio/libdeps/<PIOENV>/<Library>/...
    libdeps_dir = (
            PROJECT_DIR
            / ".pio"
            / "libdeps"
            / env.subst("$PIOENV")
    )

    add_library_tree(include_dirs, libdeps_dir)

    # Framework core and built-in framework libraries.
    for framework_dir in get_framework_directories():
        # Teensy core.
        add_include_dir(
            include_dirs,
            framework_dir / "cores" / "teensy4",
            )

        # Other Arduino core layouts.
        cores_dir = framework_dir / "cores"

        if cores_dir.is_dir():
            for core_dir in cores_dir.iterdir():
                add_include_dir(include_dirs, core_dir)

        # Framework libraries such as SPI, Wire, etc.
        add_library_tree(
            include_dirs,
            framework_dir / "libraries",
            )

        # Some mbed framework packages expose variants and cores elsewhere.
        add_include_dir(include_dirs, framework_dir)
        add_include_dir(include_dirs, framework_dir / "variants")

        variants_dir = framework_dir / "variants"

        if variants_dir.is_dir():
            for variant_dir in variants_dir.iterdir():
                add_include_dir(include_dirs, variant_dir)

    return include_dirs


def split_flags(value: str) -> list[str]:
    """Split a PlatformIO/SCons flag string safely."""
    if not value:
        return []

    return shlex.split(value)


def check_headers(target, source, env):
    del target
    del source

    shutil.rmtree(GENERATED_DIR, ignore_errors=True)
    GENERATED_DIR.mkdir(parents=True, exist_ok=True)

    compiler = env.subst("$CXX")

    compiler_flags = [
        *split_flags(projenv.subst("$CCFLAGS")),
        *split_flags(projenv.subst("$CXXFLAGS")),
        *split_flags(projenv.subst("$_CPPDEFFLAGS")),
    ]

    include_dirs = collect_include_directories()

    include_flags = [
        flag
        for directory in include_dirs
        for flag in ("-I", str(directory))
    ]

    failures = []

    print(f"\nEnvironment: {env.subst('$PIOENV')}")
    print(f"Compiler:    {compiler}")
    print(f"Headers:     {len(HEADERS)}")

    for header_name in HEADERS:
        header_path = SOURCE_DIR / header_name

        if not header_path.is_file():
            print(f"\n[NOT FOUND] {header_name}")
            failures.append(f"{header_name} (not found)")
            continue

        generated_name = header_name.replace("/", "_").replace("\\", "_")
        generated_source = GENERATED_DIR / f"{generated_name}.cpp"

        generated_source.write_text(
            f'#include "{header_path.as_posix()}"\n',
            encoding="utf-8",
        )

        object_file = GENERATED_DIR / f"{generated_name}.o"

        command = [
            compiler,
            *compiler_flags,
            *include_flags,
            "-c",
            str(generated_source),
            "-o",
            str(object_file),
        ]

        print(f"\nChecking {header_name}")

        result = subprocess.run(
            command,
            cwd=PROJECT_DIR,
            check=False,
        )

        if result.returncode != 0:
            failures.append(header_name)

    if failures:
        print("\nHeader checks failed:")

        for failure in failures:
            print(f"  - {failure}")

        env.Exit(1)

    print("\nAll headers are independently compilable.")


env.AddCustomTarget(
    name="checkheaders",
    dependencies=None,
    actions=check_headers,
    title="Check headers",
    description="Compile each project header independently",
)
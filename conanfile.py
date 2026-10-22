from conan import ConanFile
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout
from conan.tools.files import copy
import os


class NeurovectorDroneConan(ConanFile):
    name = "neurovector-drone"
    version = "0.1.0"
    license = "MIT"
    author = "NeruoVector"
    url = "https://github.com/NeruoVector/neurovector-drone"
    description = "NeuroVector 드론 하드웨어 HAL (C/C++ + Conan 2)"
    topics = ("drone", "hal", "embedded", "flight-controller")
    settings = "os", "compiler", "build_type", "arch"
    options = {"shared": [True, False], "fPIC": [True, False]}
    default_options = {"shared": False, "fPIC": True}
    exports_sources = "CMakeLists.txt", "src/*", "include/*", "firmware/*", "app/*", "tests/*"

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    def configure(self):
        if self.options.shared:
            self.options.rm_safe("fPIC")

    def layout(self):
        cmake_layout(self)

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()
        tc = CMakeToolchain(self)
        tc.variables["NEUROVECTOR_BUILD_TESTS"] = False
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()
        copy(self, "LICENSE", src=self.source_folder, dst=os.path.join(self.package_folder, "licenses"))

    def package_info(self):
        self.cpp_info.set_property("cmake_file_name", "neurovector-drone")
        self.cpp_info.set_property("cmake_target_name", "neurovector::drone_hal")

        self.cpp_info.components["hal"].set_property(
            "cmake_target_name", "neurovector::drone_hal"
        )
        self.cpp_info.components["hal"].libs = ["drone_hal"]

        self.cpp_info.components["pwm"].set_property(
            "cmake_target_name", "neurovector::nv_pwm"
        )
        self.cpp_info.components["pwm"].libs = ["nv_pwm"]

        self.cpp_info.components["mission"].set_property(
            "cmake_target_name", "neurovector::nv_mission"
        )
        self.cpp_info.components["mission"].libs = ["nv_mission"]
        self.cpp_info.components["avoidance"].set_property(
            "cmake_target_name", "neurovector::nv_avoidance"
        )
        self.cpp_info.components["avoidance"].libs = ["nv_avoidance"]

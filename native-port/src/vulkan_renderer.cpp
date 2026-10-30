#include "vulkan_renderer.hpp"
#include "utf8_path.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <SDL3/SDL_filesystem.h>
#include <filesystem>
#include <fstream>
#include <limits>
#include <new>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace ac5 {

namespace {

struct DrawPushConstants {
    Mat4 view_projection_model;
    NormalMatrix normal_matrix;
    std::array<float, 4> lighting;
};

struct DrawSubmission {
    const DrawItem* item;
    Mat4 world;
    Mat4 view_projection_model;
    float depth;
    bool transparent;
};

template <typename Callback>
class ScopeExit {
public:
    explicit ScopeExit(Callback callback) : callback_(std::move(callback)) {}
    ~ScopeExit() { if (active_) callback_(); }
    ScopeExit(const ScopeExit&) = delete;
    ScopeExit& operator=(const ScopeExit&) = delete;
    void release() noexcept { active_ = false; }

private:
    Callback callback_;
    bool active_ = true;
};

bool finite_matrix(const Mat4& matrix) {
    return std::all_of(matrix.value.begin(), matrix.value.end(),
                       [](float value) { return std::isfinite(value); });
}

float projected_depth(const Mat4& view_projection, Vec3 position) {
    const auto& matrix = view_projection.value;
    const double clip_z = matrix[2] * position.x + matrix[6] * position.y +
                          matrix[10] * position.z + matrix[14];
    const double clip_w = matrix[3] * position.x + matrix[7] * position.y +
                          matrix[11] * position.z + matrix[15];
    if (!std::isfinite(clip_z) || !std::isfinite(clip_w) || std::abs(clip_w) < 1.0e-12)
        return 0.0f;
    const double depth = clip_z / clip_w;
    return std::isfinite(depth) ? static_cast<float>(depth) : 0.0f;
}

static_assert(std::is_standard_layout_v<DrawPushConstants>);
static_assert(sizeof(NormalMatrix) == 48);
static_assert(offsetof(DrawPushConstants, normal_matrix) == 64);
static_assert(offsetof(DrawPushConstants, lighting) == 112);
static_assert(sizeof(DrawPushConstants) == 128);

template <typename T>
T vk_struct(VkStructureType type) {
    T value{};
    value.sType = type;
    return value;
}

bool has_extension(VkPhysicalDevice device, const char* name) {
    std::uint32_t count = 0;
    if (vkEnumerateDeviceExtensionProperties(device, nullptr, &count, nullptr) != VK_SUCCESS)
        return false;
    std::vector<VkExtensionProperties> extensions(count);
    if (vkEnumerateDeviceExtensionProperties(device, nullptr, &count, extensions.data()) != VK_SUCCESS)
        return false;
    return std::any_of(extensions.begin(), extensions.end(), [name](const auto& ext) {
        return std::strcmp(ext.extensionName, name) == 0;
    });
}

bool has_instance_extension(const char* name) {
    std::uint32_t count = 0;
    if (vkEnumerateInstanceExtensionProperties(nullptr, &count, nullptr) != VK_SUCCESS)
        return false;
    std::vector<VkExtensionProperties> extensions(count);
    if (vkEnumerateInstanceExtensionProperties(nullptr, &count, extensions.data()) != VK_SUCCESS)
        return false;
    return std::any_of(extensions.begin(), extensions.end(), [name](const auto& ext) {
        return std::strcmp(ext.extensionName, name) == 0;
    });
}

constexpr std::array<VkFormat, 3> supported_swapchain_formats{
    VK_FORMAT_B8G8R8A8_SRGB, VK_FORMAT_R8G8B8A8_SRGB,
    VK_FORMAT_A8B8G8R8_SRGB_PACK32};

bool supports_swapchain(VkPhysicalDevice device, VkSurfaceKHR surface) {
    VkSurfaceCapabilitiesKHR capabilities{};
    if (vkGetPhysicalDeviceSurfaceCapabilitiesKHR(device, surface, &capabilities) != VK_SUCCESS ||
        (capabilities.supportedUsageFlags & VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT) == 0)
        return false;

    std::uint32_t format_count = 0, mode_count = 0;
    if (vkGetPhysicalDeviceSurfaceFormatsKHR(device, surface, &format_count, nullptr) != VK_SUCCESS ||
        vkGetPhysicalDeviceSurfacePresentModesKHR(device, surface, &mode_count, nullptr) != VK_SUCCESS ||
        format_count == 0 || mode_count == 0)
        return false;
    std::vector<VkSurfaceFormatKHR> formats(format_count);
    if (vkGetPhysicalDeviceSurfaceFormatsKHR(device, surface, &format_count,
                                               formats.data()) != VK_SUCCESS)
        return false;
    const bool accepts_any_srgb = format_count == 1 &&
        formats[0].format == VK_FORMAT_UNDEFINED &&
        formats[0].colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR;
    return accepts_any_srgb || std::any_of(formats.begin(), formats.end(),
        [](const VkSurfaceFormatKHR& format) {
            return format.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR &&
                std::find(supported_swapchain_formats.begin(),
                          supported_swapchain_formats.end(), format.format) !=
                    supported_swapchain_formats.end();
        });
}

bool read_shader(const char* name, std::vector<std::uint32_t>& code, std::string& error) {
    const char* base = SDL_GetBasePath();
    if (!base) {
        error = SDL_GetError();
        return false;
    }
    const std::filesystem::path path = path_from_utf8(base) / "shaders" / name;
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) {
        error = "cannot open shader: " + path_to_utf8(path);
        return false;
    }
    const auto size = file.tellg();
    if (size <= 0 || size % static_cast<std::streamoff>(sizeof(std::uint32_t)) != 0) {
        error = "invalid SPIR-V file size: " + path_to_utf8(path);
        return false;
    }
    const std::streamoff byte_size = static_cast<std::streamoff>(size);
    const auto word_count = static_cast<std::uintmax_t>(byte_size) / sizeof(std::uint32_t);
    if (word_count > code.max_size() ||
        byte_size > std::numeric_limits<std::streamsize>::max()) {
        error = "SPIR-V file exceeds the platform limit: " + path_to_utf8(path);
        return false;
    }
    try {
        code.resize(static_cast<std::size_t>(word_count));
    } catch (const std::bad_alloc&) {
        error = "SPIR-V allocation failed: " + path_to_utf8(path);
        return false;
    } catch (const std::length_error&) {
        error = "SPIR-V file exceeds the platform limit: " + path_to_utf8(path);
        return false;
    }
    file.seekg(0);
    if (!file.read(reinterpret_cast<char*>(code.data()),
                   static_cast<std::streamsize>(byte_size))) {
        error = "cannot read shader: " + path_to_utf8(path);
        return false;
    }
    if (code.front() != 0x07230203u) {
        error = "invalid SPIR-V magic number: " + path_to_utf8(path);
        return false;
    }
    return true;
}

} // namespace

VulkanRenderer::~VulkanRenderer() {
    cleanup();
}

bool VulkanRenderer::fail(const char* operation, VkResult result) {
    error_ = operation;
    error_ += " failed (VkResult ";
    error_ += std::to_string(static_cast<int>(result));
    error_ += ')';
    return false;
}

bool VulkanRenderer::initialize(SDL_Window* window) {
    error_.clear();
    if (window_ != nullptr || device_ != VK_NULL_HANDLE) {
        error_ = "Vulkan renderer is already initialized";
        return false;
    }
    if (window == nullptr) {
        error_ = "cannot initialize Vulkan renderer without an SDL window";
        return false;
    }
    window_ = window;
    return create_instance() && create_surface() && choose_device() && create_device() &&
           create_swapchain() && create_frames();
}

bool VulkanRenderer::create_instance() {
    Uint32 count = 0;
    const char* const* extensions = SDL_Vulkan_GetInstanceExtensions(&count);
    if (!extensions) {
        error_ = SDL_GetError();
        return false;
    }

    auto app = vk_struct<VkApplicationInfo>(VK_STRUCTURE_TYPE_APPLICATION_INFO);
    app.pApplicationName = "AC5 Native";
    app.applicationVersion = VK_MAKE_API_VERSION(0, 0, 1, 0);
    app.pEngineName = "AC5 Native Renderer";
    app.engineVersion = VK_MAKE_API_VERSION(0, 0, 1, 0);
    app.apiVersion = VK_API_VERSION_1_3;

    auto info = vk_struct<VkInstanceCreateInfo>(VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO);
    info.pApplicationInfo = &app;
    std::vector<const char*> enabled_extensions(extensions, extensions + count);
    const bool portability_enumeration_enabled = std::any_of(
        enabled_extensions.begin(), enabled_extensions.end(), [](const char* extension) {
            return std::strcmp(extension, VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME) == 0;
        });
    const bool portability_enumeration_available =
        has_instance_extension(VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME);
    if (portability_enumeration_available && !portability_enumeration_enabled)
        enabled_extensions.push_back(VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME);
    if (portability_enumeration_available)
        info.flags |= VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR;
    info.enabledExtensionCount = static_cast<std::uint32_t>(enabled_extensions.size());
    info.ppEnabledExtensionNames = enabled_extensions.data();
    const VkResult result = vkCreateInstance(&info, nullptr, &instance_);
    return result == VK_SUCCESS || fail("vkCreateInstance", result);
}

bool VulkanRenderer::create_surface() {
    if (SDL_Vulkan_CreateSurface(window_, instance_, nullptr, &surface_)) return true;
    error_ = SDL_GetError();
    return false;
}

bool VulkanRenderer::choose_device() {
    std::uint32_t count = 0;
    VkResult result = vkEnumeratePhysicalDevices(instance_, &count, nullptr);
    if (result != VK_SUCCESS) return fail("vkEnumeratePhysicalDevices", result);
    if (count == 0) {
        error_ = "no Vulkan physical devices found";
        return false;
    }
    std::vector<VkPhysicalDevice> devices(count);
    result = vkEnumeratePhysicalDevices(instance_, &count, devices.data());
    if (result != VK_SUCCESS) return fail("vkEnumeratePhysicalDevices", result);

    int best_score = std::numeric_limits<int>::min();
    for (VkPhysicalDevice candidate : devices) {
        VkPhysicalDeviceProperties properties{};
        vkGetPhysicalDeviceProperties(candidate, &properties);
        if (properties.apiVersion < VK_API_VERSION_1_3 ||
            !has_extension(candidate, VK_KHR_SWAPCHAIN_EXTENSION_NAME) ||
            !supports_swapchain(candidate, surface_)) continue;

        auto features13 = vk_struct<VkPhysicalDeviceVulkan13Features>(
            VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES);
        auto features = vk_struct<VkPhysicalDeviceFeatures2>(
            VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2);
        features.pNext = &features13;
        vkGetPhysicalDeviceFeatures2(candidate, &features);
        if (!features13.dynamicRendering || !features13.synchronization2) continue;

        std::uint32_t queue_count = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(candidate, &queue_count, nullptr);
        std::vector<VkQueueFamilyProperties> queues(queue_count);
        vkGetPhysicalDeviceQueueFamilyProperties(candidate, &queue_count, queues.data());
        std::uint32_t graphics = UINT32_MAX, present = UINT32_MAX;
        std::uint32_t first_graphics = UINT32_MAX, first_present = UINT32_MAX;
        for (std::uint32_t i = 0; i < queue_count; ++i) {
            if (queues[i].queueCount && (queues[i].queueFlags & VK_QUEUE_GRAPHICS_BIT)) {
                if (first_graphics == UINT32_MAX) first_graphics = i;
            }
            VkBool32 can_present = VK_FALSE;
            if (queues[i].queueCount &&
                vkGetPhysicalDeviceSurfaceSupportKHR(candidate, i, surface_, &can_present) == VK_SUCCESS &&
                can_present) {
                if (first_present == UINT32_MAX) first_present = i;
                if (queues[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) {
                    graphics = present = i;
                    break;
                }
            }
        }
        if (graphics == UINT32_MAX) {
            graphics = first_graphics;
            present = first_present;
        }
        if (graphics != UINT32_MAX && present != UINT32_MAX) {
            int score = 0;
            switch (properties.deviceType) {
                case VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU: score = 1000; break;
                case VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU: score = 800; break;
                case VK_PHYSICAL_DEVICE_TYPE_VIRTUAL_GPU: score = 600; break;
                case VK_PHYSICAL_DEVICE_TYPE_CPU: score = 200; break;
                default: score = 400; break;
            }
            if (graphics == present) score += 100;
            score += static_cast<int>(properties.limits.maxImageDimension2D / 4096u);
            if (score <= best_score) continue;

            best_score = score;
            physical_device_ = candidate;
            graphics_family_ = graphics;
            present_family_ = present;
            portability_subset_supported_ =
                has_extension(candidate, "VK_KHR_portability_subset");
            sampler_anisotropy_supported_ = features.features.samplerAnisotropy;
            max_sampler_anisotropy_ = properties.limits.maxSamplerAnisotropy;
        }
    }
    if (physical_device_ != VK_NULL_HANDLE) return true;
    error_ = "no Vulkan 1.3 device with dynamic rendering, synchronization2, and a supported sRGB swapchain";
    return false;
}

bool VulkanRenderer::create_device() {
    const float priority = 1.0f;
    std::array<std::uint32_t, 2> families{graphics_family_, present_family_};
    const std::uint32_t family_count = graphics_family_ == present_family_ ? 1u : 2u;
    std::array<VkDeviceQueueCreateInfo, 2> queue_infos{};
    for (std::uint32_t i = 0; i < family_count; ++i) {
        queue_infos[i] = vk_struct<VkDeviceQueueCreateInfo>(
            VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO);
        queue_infos[i].queueFamilyIndex = families[i];
        queue_infos[i].queueCount = 1;
        queue_infos[i].pQueuePriorities = &priority;
    }

    auto features = vk_struct<VkPhysicalDeviceVulkan13Features>(
        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES);
    features.dynamicRendering = VK_TRUE;
    features.synchronization2 = VK_TRUE;
    VkPhysicalDeviceFeatures enabled_features{};
    enabled_features.samplerAnisotropy = sampler_anisotropy_supported_;
    std::vector<const char*> extensions{VK_KHR_SWAPCHAIN_EXTENSION_NAME};
    if (portability_subset_supported_)
        extensions.push_back("VK_KHR_portability_subset");
    auto info = vk_struct<VkDeviceCreateInfo>(VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO);
    info.pNext = &features;
    info.pEnabledFeatures = &enabled_features;
    info.queueCreateInfoCount = family_count;
    info.pQueueCreateInfos = queue_infos.data();
    info.enabledExtensionCount = static_cast<std::uint32_t>(extensions.size());
    info.ppEnabledExtensionNames = extensions.data();
    VkResult result = vkCreateDevice(physical_device_, &info, nullptr, &device_);
    if (result != VK_SUCCESS) return fail("vkCreateDevice", result);
    vkGetDeviceQueue(device_, graphics_family_, 0, &graphics_queue_);
    vkGetDeviceQueue(device_, present_family_, 0, &present_queue_);

    auto pool = vk_struct<VkCommandPoolCreateInfo>(VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO);
    pool.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    pool.queueFamilyIndex = graphics_family_;
    result = vkCreateCommandPool(device_, &pool, nullptr, &command_pool_);
    return result == VK_SUCCESS || fail("vkCreateCommandPool", result);
}

bool VulkanRenderer::create_swapchain() {
    VkSurfaceCapabilitiesKHR caps{};
    VkResult result = vkGetPhysicalDeviceSurfaceCapabilitiesKHR(physical_device_, surface_, &caps);
    if (result != VK_SUCCESS) return fail("vkGetPhysicalDeviceSurfaceCapabilitiesKHR", result);

    std::uint32_t format_count = 0;
    result = vkGetPhysicalDeviceSurfaceFormatsKHR(physical_device_, surface_, &format_count, nullptr);
    if (result != VK_SUCCESS || format_count == 0) return fail("surface formats", result);
    std::vector<VkSurfaceFormatKHR> formats(format_count);
    result = vkGetPhysicalDeviceSurfaceFormatsKHR(physical_device_, surface_, &format_count, formats.data());
    if (result != VK_SUCCESS) return fail("vkGetPhysicalDeviceSurfaceFormatsKHR", result);
    auto chosen = formats.front();
    const bool any_format = format_count == 1 &&
                            chosen.format == VK_FORMAT_UNDEFINED;
    if (any_format) chosen.format = VK_FORMAT_B8G8R8A8_SRGB;
    bool found_srgb_format = any_format &&
        chosen.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR;
    for (const VkFormat preferred : supported_swapchain_formats) {
        const auto match = std::find_if(formats.begin(), formats.end(), [preferred](const auto& format) {
            return format.format == preferred &&
                   format.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR;
        });
        if (match != formats.end()) {
            chosen = *match;
            found_srgb_format = true;
            break;
        }
    }
    if (!found_srgb_format) {
        error_ = "surface does not provide a supported sRGB nonlinear swapchain format";
        return false;
    }

    VkExtent2D extent = caps.currentExtent;
    if (extent.width == std::numeric_limits<std::uint32_t>::max()) {
        int width = 0, height = 0;
        SDL_GetWindowSizeInPixels(window_, &width, &height);
        if (width == 0 || height == 0) {
            error_ = "window is minimized";
            return false;
        }
        extent.width = std::clamp(static_cast<std::uint32_t>(width),
                                  caps.minImageExtent.width, caps.maxImageExtent.width);
        extent.height = std::clamp(static_cast<std::uint32_t>(height),
                                   caps.minImageExtent.height, caps.maxImageExtent.height);
    }
    if (extent.width == 0 || extent.height == 0) {
        error_ = "surface has no drawable extent";
        return false;
    }
    if ((caps.supportedUsageFlags & VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT) == 0) {
        error_ = "surface images cannot be used as color attachments";
        return false;
    }
    std::uint32_t image_count = caps.minImageCount + 1;
    if (caps.maxImageCount && image_count > caps.maxImageCount) image_count = caps.maxImageCount;

    const std::array<std::uint32_t, 2> families{graphics_family_, present_family_};
    auto info = vk_struct<VkSwapchainCreateInfoKHR>(VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR);
    info.surface = surface_;
    info.minImageCount = image_count;
    info.imageFormat = chosen.format;
    info.imageColorSpace = chosen.colorSpace;
    info.imageExtent = extent;
    info.imageArrayLayers = 1;
    info.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    if (graphics_family_ != present_family_) {
        info.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
        info.queueFamilyIndexCount = 2;
        info.pQueueFamilyIndices = families.data();
    } else {
        info.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    }
    info.preTransform = caps.currentTransform;
    constexpr std::array<VkCompositeAlphaFlagBitsKHR, 4> alpha_modes{
        VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR,
        VK_COMPOSITE_ALPHA_PRE_MULTIPLIED_BIT_KHR,
        VK_COMPOSITE_ALPHA_POST_MULTIPLIED_BIT_KHR,
        VK_COMPOSITE_ALPHA_INHERIT_BIT_KHR};
    for (const auto mode : alpha_modes) {
        if (caps.supportedCompositeAlpha & mode) {
            info.compositeAlpha = mode;
            break;
        }
    }
    info.presentMode = VK_PRESENT_MODE_FIFO_KHR;
    info.clipped = VK_TRUE;
    result = vkCreateSwapchainKHR(device_, &info, nullptr, &swapchain_);
    if (result != VK_SUCCESS) return fail("vkCreateSwapchainKHR", result);
    swapchain_format_ = chosen.format;
    swapchain_extent_ = extent;

    result = vkGetSwapchainImagesKHR(device_, swapchain_, &image_count, nullptr);
    if (result != VK_SUCCESS) return fail("vkGetSwapchainImagesKHR", result);
    images_.resize(image_count);
    result = vkGetSwapchainImagesKHR(device_, swapchain_, &image_count, images_.data());
    if (result != VK_SUCCESS) return fail("vkGetSwapchainImagesKHR", result);
    image_views_.resize(image_count, VK_NULL_HANDLE);
    render_finished_.resize(image_count, VK_NULL_HANDLE);
    image_fences_.resize(image_count, VK_NULL_HANDLE);
    image_initialized_.resize(image_count, false);
    for (std::size_t i = 0; i < images_.size(); ++i) {
        auto view = vk_struct<VkImageViewCreateInfo>(VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO);
        view.image = images_[i];
        view.viewType = VK_IMAGE_VIEW_TYPE_2D;
        view.format = swapchain_format_;
        view.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        result = vkCreateImageView(device_, &view, nullptr, &image_views_[i]);
        if (result != VK_SUCCESS) return fail("vkCreateImageView", result);
        auto semaphore = vk_struct<VkSemaphoreCreateInfo>(VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO);
        result = vkCreateSemaphore(device_, &semaphore, nullptr, &render_finished_[i]);
        if (result != VK_SUCCESS) return fail("vkCreateSemaphore(present)", result);
    }
    if (!create_depth_resources()) return false;

    command_buffers_.resize(image_count, VK_NULL_HANDLE);
    auto alloc = vk_struct<VkCommandBufferAllocateInfo>(VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO);
    alloc.commandPool = command_pool_;
    alloc.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    alloc.commandBufferCount = image_count;
    result = vkAllocateCommandBuffers(device_, &alloc, command_buffers_.data());
    if (result != VK_SUCCESS) return fail("vkAllocateCommandBuffers", result);
    return create_pipeline();
}

bool VulkanRenderer::create_depth_resources() {
    constexpr std::array<VkFormat, 3> candidates{
        VK_FORMAT_D32_SFLOAT, VK_FORMAT_D24_UNORM_S8_UINT, VK_FORMAT_D16_UNORM};
    for (VkFormat candidate : candidates) {
        VkFormatProperties properties{};
        vkGetPhysicalDeviceFormatProperties(physical_device_, candidate, &properties);
        if (properties.optimalTilingFeatures & VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT) {
            depth_format_ = candidate;
            break;
        }
    }
    if (depth_format_ == VK_FORMAT_UNDEFINED) {
        error_ = "device has no supported depth attachment format";
        return false;
    }
    depth_images_.resize(images_.size(), VK_NULL_HANDLE);
    depth_memories_.resize(images_.size(), VK_NULL_HANDLE);
    depth_views_.resize(images_.size(), VK_NULL_HANDLE);
    for (std::size_t i = 0; i < images_.size(); ++i) {
        auto image = vk_struct<VkImageCreateInfo>(VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO);
        image.imageType = VK_IMAGE_TYPE_2D;
        image.format = depth_format_;
        image.extent = {swapchain_extent_.width, swapchain_extent_.height, 1};
        image.mipLevels = 1;
        image.arrayLayers = 1;
        image.samples = VK_SAMPLE_COUNT_1_BIT;
        image.tiling = VK_IMAGE_TILING_OPTIMAL;
        image.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
        image.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        VkResult result = vkCreateImage(device_, &image, nullptr, &depth_images_[i]);
        if (result != VK_SUCCESS) return fail("vkCreateImage(depth)", result);
        VkMemoryRequirements requirements{};
        vkGetImageMemoryRequirements(device_, depth_images_[i], &requirements);
        std::uint32_t memory_type = 0;
        if (!find_memory_type(requirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
                              memory_type)) return false;
        auto allocation = vk_struct<VkMemoryAllocateInfo>(VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO);
        allocation.allocationSize = requirements.size;
        allocation.memoryTypeIndex = memory_type;
        result = vkAllocateMemory(device_, &allocation, nullptr, &depth_memories_[i]);
        if (result != VK_SUCCESS) return fail("vkAllocateMemory(depth)", result);
        result = vkBindImageMemory(device_, depth_images_[i], depth_memories_[i], 0);
        if (result != VK_SUCCESS) return fail("vkBindImageMemory(depth)", result);
        auto view = vk_struct<VkImageViewCreateInfo>(VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO);
        view.image = depth_images_[i];
        view.viewType = VK_IMAGE_VIEW_TYPE_2D;
        view.format = depth_format_;
        view.subresourceRange = {VK_IMAGE_ASPECT_DEPTH_BIT, 0, 1, 0, 1};
        result = vkCreateImageView(device_, &view, nullptr, &depth_views_[i]);
        if (result != VK_SUCCESS) return fail("vkCreateImageView(depth)", result);
    }
    return true;
}

bool VulkanRenderer::create_pipeline() {
    std::vector<std::uint32_t> vertex_code, fragment_code, environment_fragment_code;
    if (!read_shader("mesh.vert.spv", vertex_code, error_) ||
        !read_shader("mesh.frag.spv", fragment_code, error_) ||
        !read_shader("environment.frag.spv", environment_fragment_code, error_)) return false;
    auto create_shader = [this](const std::vector<std::uint32_t>& code, VkShaderModule& module) {
        auto info = vk_struct<VkShaderModuleCreateInfo>(VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO);
        info.codeSize = code.size() * sizeof(std::uint32_t);
        info.pCode = code.data();
        const VkResult result = vkCreateShaderModule(device_, &info, nullptr, &module);
        return result == VK_SUCCESS || fail("vkCreateShaderModule", result);
    };
    VkShaderModule vertex = VK_NULL_HANDLE, fragment = VK_NULL_HANDLE;
    VkShaderModule environment_fragment = VK_NULL_HANDLE;
    if (!create_shader(vertex_code, vertex) || !create_shader(fragment_code, fragment) ||
        !create_shader(environment_fragment_code, environment_fragment)) {
        if (environment_fragment) vkDestroyShaderModule(device_, environment_fragment, nullptr);
        if (vertex) vkDestroyShaderModule(device_, vertex, nullptr);
        if (fragment) vkDestroyShaderModule(device_, fragment, nullptr);
        return false;
    }

    if (texture_set_layout_ == VK_NULL_HANDLE) {
        auto binding = VkDescriptorSetLayoutBinding{};
        binding.binding = 0;
        binding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        binding.descriptorCount = 1;
        binding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        auto descriptor_layout = vk_struct<VkDescriptorSetLayoutCreateInfo>(
            VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO);
        descriptor_layout.bindingCount = 1;
        descriptor_layout.pBindings = &binding;
        const VkResult layout_result = vkCreateDescriptorSetLayout(
            device_, &descriptor_layout, nullptr, &texture_set_layout_);
        if (layout_result != VK_SUCCESS) {
            vkDestroyShaderModule(device_, environment_fragment, nullptr);
            vkDestroyShaderModule(device_, fragment, nullptr);
            vkDestroyShaderModule(device_, vertex, nullptr);
            return fail("vkCreateDescriptorSetLayout(texture)", layout_result);
        }
    }

    auto layout_info = vk_struct<VkPipelineLayoutCreateInfo>(VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO);
    layout_info.setLayoutCount = 1;
    layout_info.pSetLayouts = &texture_set_layout_;
    VkPushConstantRange draw_constants{};
    draw_constants.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    draw_constants.offset = 0;
    draw_constants.size = sizeof(DrawPushConstants);
    layout_info.pushConstantRangeCount = 1;
    layout_info.pPushConstantRanges = &draw_constants;
    VkResult result = vkCreatePipelineLayout(device_, &layout_info, nullptr, &pipeline_layout_);
    if (result != VK_SUCCESS) {
        vkDestroyShaderModule(device_, environment_fragment, nullptr);
        vkDestroyShaderModule(device_, fragment, nullptr);
        vkDestroyShaderModule(device_, vertex, nullptr);
        return fail("vkCreatePipelineLayout", result);
    }

    std::array<VkPipelineShaderStageCreateInfo, 2> stages{};
    stages[0] = vk_struct<VkPipelineShaderStageCreateInfo>(VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO);
    stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
    stages[0].module = vertex;
    stages[0].pName = "main";
    stages[1] = vk_struct<VkPipelineShaderStageCreateInfo>(VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO);
    stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    stages[1].module = fragment;
    stages[1].pName = "main";

    auto vertex_input = vk_struct<VkPipelineVertexInputStateCreateInfo>(
        VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO);
    static_assert(std::is_standard_layout_v<Vertex>);
    VkVertexInputBindingDescription vertex_binding{0, sizeof(Vertex), VK_VERTEX_INPUT_RATE_VERTEX};
    std::array<VkVertexInputAttributeDescription, 9> attributes{{
        {0, 0, VK_FORMAT_R32G32B32_SFLOAT, static_cast<std::uint32_t>(offsetof(Vertex, position))},
        {1, 0, VK_FORMAT_R32G32B32_SFLOAT, static_cast<std::uint32_t>(offsetof(Vertex, color))},
        {2, 0, VK_FORMAT_R32G32_SFLOAT, static_cast<std::uint32_t>(offsetof(Vertex, uv))},
        {3, 0, VK_FORMAT_R32G32B32_SFLOAT, static_cast<std::uint32_t>(offsetof(Vertex, normal))},
        {4, 0, VK_FORMAT_R32_SFLOAT, static_cast<std::uint32_t>(offsetof(Vertex, alpha))},
        {5, 0, VK_FORMAT_R32G32_SFLOAT, static_cast<std::uint32_t>(offsetof(Vertex, environment_uv))},
        {6, 0, VK_FORMAT_R32G32B32_SFLOAT, static_cast<std::uint32_t>(offsetof(Vertex, environment_color))},
        {7, 0, VK_FORMAT_R32_SFLOAT, static_cast<std::uint32_t>(offsetof(Vertex, environment_alpha))},
        {8, 0, VK_FORMAT_R32_SFLOAT, static_cast<std::uint32_t>(offsetof(Vertex, light_response))}}};
    vertex_input.vertexBindingDescriptionCount = 1;
    vertex_input.pVertexBindingDescriptions = &vertex_binding;
    vertex_input.vertexAttributeDescriptionCount = static_cast<std::uint32_t>(attributes.size());
    vertex_input.pVertexAttributeDescriptions = attributes.data();
    auto assembly = vk_struct<VkPipelineInputAssemblyStateCreateInfo>(
        VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO);
    assembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    auto viewport = vk_struct<VkPipelineViewportStateCreateInfo>(
        VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO);
    viewport.viewportCount = 1;
    viewport.scissorCount = 1;
    auto raster = vk_struct<VkPipelineRasterizationStateCreateInfo>(
        VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO);
    raster.polygonMode = VK_POLYGON_MODE_FILL;
    raster.cullMode = VK_CULL_MODE_NONE;
    raster.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    raster.lineWidth = 1.0f;
    auto multisample = vk_struct<VkPipelineMultisampleStateCreateInfo>(
        VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO);
    multisample.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
    auto depth = vk_struct<VkPipelineDepthStencilStateCreateInfo>(
        VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO);
    depth.depthTestEnable = VK_TRUE;
    depth.depthWriteEnable = VK_TRUE;
    depth.depthCompareOp = VK_COMPARE_OP_LESS;
    auto blend_attachment = VkPipelineColorBlendAttachmentState{};
    blend_attachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                                      VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    auto blend = vk_struct<VkPipelineColorBlendStateCreateInfo>(
        VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO);
    blend.attachmentCount = 1;
    blend.pAttachments = &blend_attachment;
    const std::array<VkDynamicState, 2> dynamics{VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
    auto dynamic = vk_struct<VkPipelineDynamicStateCreateInfo>(
        VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO);
    dynamic.dynamicStateCount = static_cast<std::uint32_t>(dynamics.size());
    dynamic.pDynamicStates = dynamics.data();
    auto rendering = vk_struct<VkPipelineRenderingCreateInfo>(VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO);
    rendering.colorAttachmentCount = 1;
    rendering.pColorAttachmentFormats = &swapchain_format_;
    rendering.depthAttachmentFormat = depth_format_;
    auto pipeline_info = vk_struct<VkGraphicsPipelineCreateInfo>(VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO);
    pipeline_info.pNext = &rendering;
    pipeline_info.stageCount = static_cast<std::uint32_t>(stages.size());
    pipeline_info.pStages = stages.data();
    pipeline_info.pVertexInputState = &vertex_input;
    pipeline_info.pInputAssemblyState = &assembly;
    pipeline_info.pViewportState = &viewport;
    pipeline_info.pRasterizationState = &raster;
    pipeline_info.pMultisampleState = &multisample;
    pipeline_info.pDepthStencilState = &depth;
    pipeline_info.pColorBlendState = &blend;
    pipeline_info.pDynamicState = &dynamic;
    pipeline_info.layout = pipeline_layout_;
    result = vkCreateGraphicsPipelines(device_, VK_NULL_HANDLE, 1, &pipeline_info, nullptr, &pipeline_);
    if (result == VK_SUCCESS) {
        depth.depthWriteEnable = VK_FALSE;
        blend_attachment.blendEnable = VK_TRUE;
        blend_attachment.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
        blend_attachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        blend_attachment.colorBlendOp = VK_BLEND_OP_ADD;
        blend_attachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
        blend_attachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        blend_attachment.alphaBlendOp = VK_BLEND_OP_ADD;
        result = vkCreateGraphicsPipelines(device_, VK_NULL_HANDLE, 1, &pipeline_info,
                                           nullptr, &transparent_pipeline_);
    }
    if (result == VK_SUCCESS) {
        depth.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
        stages[1].module = environment_fragment;
        result = vkCreateGraphicsPipelines(device_, VK_NULL_HANDLE, 1, &pipeline_info,
                                           nullptr, &environment_pipeline_);
    }
    vkDestroyShaderModule(device_, environment_fragment, nullptr);
    vkDestroyShaderModule(device_, fragment, nullptr);
    vkDestroyShaderModule(device_, vertex, nullptr);
    if (result != VK_SUCCESS) {
        destroy_pipeline();
        return fail("vkCreateGraphicsPipelines", result);
    }
    return true;
}

bool VulkanRenderer::create_frames() {
    VkResult result;
    for (auto& frame : frames_) {
        auto semaphore = vk_struct<VkSemaphoreCreateInfo>(VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO);
        result = vkCreateSemaphore(device_, &semaphore, nullptr, &frame.image_available);
        if (result != VK_SUCCESS) return fail("vkCreateSemaphore", result);
        auto fence = vk_struct<VkFenceCreateInfo>(VK_STRUCTURE_TYPE_FENCE_CREATE_INFO);
        fence.flags = VK_FENCE_CREATE_SIGNALED_BIT;
        result = vkCreateFence(device_, &fence, nullptr, &frame.in_flight);
        if (result != VK_SUCCESS) return fail("vkCreateFence", result);
    }
    return true;
}

bool VulkanRenderer::find_memory_type(std::uint32_t mask, VkMemoryPropertyFlags properties,
                                     std::uint32_t& type_index) {
    VkPhysicalDeviceMemoryProperties memory_properties{};
    vkGetPhysicalDeviceMemoryProperties(physical_device_, &memory_properties);
    for (std::uint32_t i = 0; i < memory_properties.memoryTypeCount; ++i) {
        if ((mask & (1u << i)) &&
            (memory_properties.memoryTypes[i].propertyFlags & properties) == properties) {
            type_index = i;
            return true;
        }
    }
    error_ = "no Vulkan memory type satisfies the requested properties";
    return false;
}

bool VulkanRenderer::create_buffer(VkDeviceSize size, VkBufferUsageFlags usage,
                                   VkMemoryPropertyFlags properties, VkBuffer& buffer,
                                   VkDeviceMemory& memory,
                                   VkMemoryPropertyFlags* selected_properties) {
    if (size == 0) {
        error_ = "cannot create an empty Vulkan buffer";
        return false;
    }
    auto info = vk_struct<VkBufferCreateInfo>(VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO);
    info.size = size;
    info.usage = usage;
    info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    VkResult result = vkCreateBuffer(device_, &info, nullptr, &buffer);
    if (result != VK_SUCCESS) return fail("vkCreateBuffer", result);
    VkMemoryRequirements requirements{};
    vkGetBufferMemoryRequirements(device_, buffer, &requirements);
    std::uint32_t memory_type = 0;
    if (!find_memory_type(requirements.memoryTypeBits, properties, memory_type)) return false;
    if (selected_properties != nullptr) {
        VkPhysicalDeviceMemoryProperties memory_properties{};
        vkGetPhysicalDeviceMemoryProperties(physical_device_, &memory_properties);
        *selected_properties = memory_properties.memoryTypes[memory_type].propertyFlags;
    }
    auto allocation = vk_struct<VkMemoryAllocateInfo>(VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO);
    allocation.allocationSize = requirements.size;
    allocation.memoryTypeIndex = memory_type;
    result = vkAllocateMemory(device_, &allocation, nullptr, &memory);
    if (result != VK_SUCCESS) return fail("vkAllocateMemory(buffer)", result);
    result = vkBindBufferMemory(device_, buffer, memory, 0);
    if (result != VK_SUCCESS) return fail("vkBindBufferMemory", result);
    return true;
}

bool VulkanRenderer::upload_texture(const TextureData& texture, GpuTexture& gpu_texture) {
    const std::uint64_t pixels = static_cast<std::uint64_t>(texture.width) * texture.height;
    if (texture.width == 0 || texture.height == 0 ||
        pixels > std::numeric_limits<std::size_t>::max() / 4u ||
        texture.rgba.size() != pixels * 4u) {
        error_ = "scene texture dimensions do not match its RGBA data";
        return false;
    }
    gpu_texture.has_transparency = false;
    for (std::size_t alpha = 3; alpha < texture.rgba.size(); alpha += 4) {
        if (texture.rgba[alpha] != 255) {
            gpu_texture.has_transparency = true;
            break;
        }
    }
    VkPhysicalDeviceProperties device_properties{};
    vkGetPhysicalDeviceProperties(physical_device_, &device_properties);
    if (texture.width > device_properties.limits.maxImageDimension2D ||
        texture.height > device_properties.limits.maxImageDimension2D ||
        texture.width > static_cast<std::uint32_t>(std::numeric_limits<std::int32_t>::max()) ||
        texture.height > static_cast<std::uint32_t>(std::numeric_limits<std::int32_t>::max())) {
        error_ = "scene texture dimensions exceed the Vulkan device limit";
        return false;
    }
    constexpr VkFormat format = VK_FORMAT_R8G8B8A8_SRGB;
    VkFormatProperties format_properties{};
    vkGetPhysicalDeviceFormatProperties(physical_device_, format, &format_properties);
    constexpr VkFormatFeatureFlags required = VK_FORMAT_FEATURE_SAMPLED_IMAGE_BIT |
                                              VK_FORMAT_FEATURE_SAMPLED_IMAGE_FILTER_LINEAR_BIT;
    if ((format_properties.optimalTilingFeatures & required) != required) {
        error_ = "device cannot linearly sample an sRGB RGBA texture";
        return false;
    }
    constexpr VkFormatFeatureFlags blit_features = VK_FORMAT_FEATURE_BLIT_SRC_BIT |
                                                   VK_FORMAT_FEATURE_BLIT_DST_BIT;
    const bool can_generate_mips =
        (format_properties.optimalTilingFeatures & (blit_features |
             VK_FORMAT_FEATURE_SAMPLED_IMAGE_FILTER_LINEAR_BIT)) ==
        (blit_features | VK_FORMAT_FEATURE_SAMPLED_IMAGE_FILTER_LINEAR_BIT);
    gpu_texture.mip_levels = can_generate_mips
        ? std::bit_width(std::max(texture.width, texture.height)) : 1u;

    auto image_info = vk_struct<VkImageCreateInfo>(VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO);
    image_info.imageType = VK_IMAGE_TYPE_2D;
    image_info.format = format;
    image_info.extent = {texture.width, texture.height, 1};
    image_info.mipLevels = gpu_texture.mip_levels;
    image_info.arrayLayers = 1;
    image_info.samples = VK_SAMPLE_COUNT_1_BIT;
    image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
    image_info.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT |
                       (gpu_texture.mip_levels > 1 ? VK_IMAGE_USAGE_TRANSFER_SRC_BIT : 0);
    image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    VkResult result = vkCreateImage(device_, &image_info, nullptr, &gpu_texture.image);
    if (result != VK_SUCCESS) return fail("vkCreateImage(texture)", result);

    VkMemoryRequirements requirements{};
    vkGetImageMemoryRequirements(device_, gpu_texture.image, &requirements);
    std::uint32_t memory_type = 0;
    if (!find_memory_type(requirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
                          memory_type)) return false;
    auto allocation = vk_struct<VkMemoryAllocateInfo>(VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO);
    allocation.allocationSize = requirements.size;
    allocation.memoryTypeIndex = memory_type;
    result = vkAllocateMemory(device_, &allocation, nullptr, &gpu_texture.memory);
    if (result != VK_SUCCESS) return fail("vkAllocateMemory(texture)", result);
    result = vkBindImageMemory(device_, gpu_texture.image, gpu_texture.memory, 0);
    if (result != VK_SUCCESS) return fail("vkBindImageMemory(texture)", result);

    VkBuffer staging_buffer = VK_NULL_HANDLE;
    VkDeviceMemory staging_memory = VK_NULL_HANDLE;
    const VkDeviceSize staging_size = texture.rgba.size();
    VkMemoryPropertyFlags staging_properties = 0;
    auto cleanup_staging = [this, &staging_buffer, &staging_memory]() {
        if (staging_buffer) vkDestroyBuffer(device_, staging_buffer, nullptr);
        if (staging_memory) vkFreeMemory(device_, staging_memory, nullptr);
    };
    if (!create_buffer(staging_size, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                       VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT, staging_buffer, staging_memory,
                       &staging_properties)) {
        cleanup_staging();
        return false;
    }
    void* mapped = nullptr;
    result = vkMapMemory(device_, staging_memory, 0, VK_WHOLE_SIZE, 0, &mapped);
    if (result != VK_SUCCESS) {
        cleanup_staging();
        return fail("vkMapMemory(texture)", result);
    }
    std::memcpy(mapped, texture.rgba.data(), texture.rgba.size());
    if ((staging_properties & VK_MEMORY_PROPERTY_HOST_COHERENT_BIT) == 0) {
        auto range = vk_struct<VkMappedMemoryRange>(VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE);
        range.memory = staging_memory;
        range.offset = 0;
        range.size = VK_WHOLE_SIZE;
        result = vkFlushMappedMemoryRanges(device_, 1, &range);
        if (result != VK_SUCCESS) {
            vkUnmapMemory(device_, staging_memory);
            cleanup_staging();
            return fail("vkFlushMappedMemoryRanges(texture)", result);
        }
    }
    vkUnmapMemory(device_, staging_memory);

    VkCommandBuffer upload = VK_NULL_HANDLE;
    auto command_alloc = vk_struct<VkCommandBufferAllocateInfo>(
        VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO);
    command_alloc.commandPool = command_pool_;
    command_alloc.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    command_alloc.commandBufferCount = 1;
    result = vkAllocateCommandBuffers(device_, &command_alloc, &upload);
    if (result != VK_SUCCESS) {
        cleanup_staging();
        return fail("vkAllocateCommandBuffers(texture)", result);
    }
    auto cleanup_upload = [this, &upload]() {
        if (upload) vkFreeCommandBuffers(device_, command_pool_, 1, &upload);
    };
    auto begin = vk_struct<VkCommandBufferBeginInfo>(VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO);
    begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    result = vkBeginCommandBuffer(upload, &begin);
    if (result != VK_SUCCESS) {
        cleanup_upload();
        cleanup_staging();
        return fail("vkBeginCommandBuffer(texture)", result);
    }
    auto to_copy = vk_struct<VkImageMemoryBarrier2>(VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2);
    to_copy.srcStageMask = VK_PIPELINE_STAGE_2_NONE;
    to_copy.srcAccessMask = VK_ACCESS_2_NONE;
    to_copy.dstStageMask = VK_PIPELINE_STAGE_2_COPY_BIT;
    to_copy.dstAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
    to_copy.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    to_copy.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    to_copy.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    to_copy.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    to_copy.image = gpu_texture.image;
    to_copy.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, gpu_texture.mip_levels, 0, 1};
    auto dependency = vk_struct<VkDependencyInfo>(VK_STRUCTURE_TYPE_DEPENDENCY_INFO);
    dependency.imageMemoryBarrierCount = 1;
    dependency.pImageMemoryBarriers = &to_copy;
    vkCmdPipelineBarrier2(upload, &dependency);

    VkBufferImageCopy copy{};
    copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
    copy.imageExtent = {texture.width, texture.height, 1};
    vkCmdCopyBufferToImage(upload, staging_buffer, gpu_texture.image,
                           VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy);
    std::int32_t mip_width = static_cast<std::int32_t>(texture.width);
    std::int32_t mip_height = static_cast<std::int32_t>(texture.height);
    for (std::uint32_t level = 1; level < gpu_texture.mip_levels; ++level) {
        auto to_blit = vk_struct<VkImageMemoryBarrier2>(VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2);
        to_blit.srcStageMask = level == 1 ? VK_PIPELINE_STAGE_2_COPY_BIT
                                          : VK_PIPELINE_STAGE_2_BLIT_BIT;
        to_blit.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
        to_blit.dstStageMask = VK_PIPELINE_STAGE_2_BLIT_BIT;
        to_blit.dstAccessMask = VK_ACCESS_2_TRANSFER_READ_BIT;
        to_blit.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        to_blit.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
        to_blit.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        to_blit.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        to_blit.image = gpu_texture.image;
        to_blit.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, level - 1, 1, 0, 1};
        dependency.pImageMemoryBarriers = &to_blit;
        vkCmdPipelineBarrier2(upload, &dependency);

        const std::int32_t next_width = std::max(1, mip_width / 2);
        const std::int32_t next_height = std::max(1, mip_height / 2);
        VkImageBlit blit{};
        blit.srcSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, level - 1, 0, 1};
        blit.srcOffsets[1] = {mip_width, mip_height, 1};
        blit.dstSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, level, 0, 1};
        blit.dstOffsets[1] = {next_width, next_height, 1};
        vkCmdBlitImage(upload, gpu_texture.image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                       gpu_texture.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                       1, &blit, VK_FILTER_LINEAR);

        auto to_sample = vk_struct<VkImageMemoryBarrier2>(VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2);
        to_sample.srcStageMask = VK_PIPELINE_STAGE_2_BLIT_BIT;
        to_sample.srcAccessMask = VK_ACCESS_2_TRANSFER_READ_BIT;
        to_sample.dstStageMask = VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT;
        to_sample.dstAccessMask = VK_ACCESS_2_SHADER_SAMPLED_READ_BIT;
        to_sample.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
        to_sample.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        to_sample.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        to_sample.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        to_sample.image = gpu_texture.image;
        to_sample.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, level - 1, 1, 0, 1};
        dependency.pImageMemoryBarriers = &to_sample;
        vkCmdPipelineBarrier2(upload, &dependency);
        mip_width = next_width;
        mip_height = next_height;
    }
    auto final_sample = vk_struct<VkImageMemoryBarrier2>(VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2);
    final_sample.srcStageMask = gpu_texture.mip_levels == 1 ? VK_PIPELINE_STAGE_2_COPY_BIT
                                                            : VK_PIPELINE_STAGE_2_BLIT_BIT;
    final_sample.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
    final_sample.dstStageMask = VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT;
    final_sample.dstAccessMask = VK_ACCESS_2_SHADER_SAMPLED_READ_BIT;
    final_sample.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    final_sample.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    final_sample.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    final_sample.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    final_sample.image = gpu_texture.image;
    final_sample.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT,
                                     gpu_texture.mip_levels - 1, 1, 0, 1};
    dependency.pImageMemoryBarriers = &final_sample;
    vkCmdPipelineBarrier2(upload, &dependency);
    result = vkEndCommandBuffer(upload);
    if (result != VK_SUCCESS) {
        cleanup_upload();
        cleanup_staging();
        return fail("vkEndCommandBuffer(texture)", result);
    }
    auto fence_info = vk_struct<VkFenceCreateInfo>(VK_STRUCTURE_TYPE_FENCE_CREATE_INFO);
    VkFence fence = VK_NULL_HANDLE;
    result = vkCreateFence(device_, &fence_info, nullptr, &fence);
    if (result != VK_SUCCESS) {
        cleanup_upload();
        cleanup_staging();
        return fail("vkCreateFence(texture)", result);
    }
    VkSubmitInfo submit{};
    submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submit.commandBufferCount = 1;
    submit.pCommandBuffers = &upload;
    result = vkQueueSubmit(graphics_queue_, 1, &submit, fence);
    if (result == VK_SUCCESS) result = vkWaitForFences(device_, 1, &fence, VK_TRUE, UINT64_MAX);
    vkDestroyFence(device_, fence, nullptr);
    cleanup_upload();
    cleanup_staging();
    if (result != VK_SUCCESS) return fail("vkQueueSubmit(texture upload)", result);

    auto view_info = vk_struct<VkImageViewCreateInfo>(VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO);
    view_info.image = gpu_texture.image;
    view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
    view_info.format = format;
    view_info.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, gpu_texture.mip_levels, 0, 1};
    result = vkCreateImageView(device_, &view_info, nullptr, &gpu_texture.view);
    if (result != VK_SUCCESS) return fail("vkCreateImageView(texture)", result);
    auto sampler_info = vk_struct<VkSamplerCreateInfo>(VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO);
    sampler_info.magFilter = VK_FILTER_LINEAR;
    sampler_info.minFilter = VK_FILTER_LINEAR;
    sampler_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
    sampler_info.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    sampler_info.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    sampler_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    sampler_info.maxLod = static_cast<float>(gpu_texture.mip_levels - 1);
    if (sampler_anisotropy_supported_) {
        sampler_info.anisotropyEnable = VK_TRUE;
        sampler_info.maxAnisotropy = std::min(max_sampler_anisotropy_, 16.0f);
    }
    result = vkCreateSampler(device_, &sampler_info, nullptr, &gpu_texture.sampler);
    if (result != VK_SUCCESS) return fail("vkCreateSampler(texture)", result);

    return true;
}

bool VulkanRenderer::create_texture_descriptors(std::vector<GpuTexture>& textures,
                                                VkDescriptorPool& descriptor_pool) {
    if (textures.empty() || textures.size() > std::numeric_limits<std::uint32_t>::max()) {
        error_ = "scene texture count is invalid";
        return false;
    }
    const auto count = static_cast<std::uint32_t>(textures.size());
    VkDescriptorPoolSize pool_size{VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, count};
    auto pool_info = vk_struct<VkDescriptorPoolCreateInfo>(VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO);
    pool_info.maxSets = count;
    pool_info.poolSizeCount = 1;
    pool_info.pPoolSizes = &pool_size;
    VkResult result = vkCreateDescriptorPool(device_, &pool_info, nullptr, &descriptor_pool);
    if (result != VK_SUCCESS) return fail("vkCreateDescriptorPool(textures)", result);
    std::vector<VkDescriptorSetLayout> layouts(count, texture_set_layout_);
    std::vector<VkDescriptorSet> sets(count, VK_NULL_HANDLE);
    auto set_alloc = vk_struct<VkDescriptorSetAllocateInfo>(VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO);
    set_alloc.descriptorPool = descriptor_pool;
    set_alloc.descriptorSetCount = count;
    set_alloc.pSetLayouts = layouts.data();
    result = vkAllocateDescriptorSets(device_, &set_alloc, sets.data());
    if (result != VK_SUCCESS) return fail("vkAllocateDescriptorSets(textures)", result);
    std::vector<VkDescriptorImageInfo> image_descriptors(count);
    std::vector<VkWriteDescriptorSet> writes(count);
    for (std::uint32_t i = 0; i < count; ++i) {
        GpuTexture& texture = textures[i];
        texture.descriptor_set = sets[i];
        image_descriptors[i].sampler = texture.sampler;
        image_descriptors[i].imageView = texture.view;
        image_descriptors[i].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[i].dstSet = sets[i];
        writes[i].dstBinding = 0;
        writes[i].descriptorCount = 1;
        writes[i].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        writes[i].pImageInfo = &image_descriptors[i];
    }
    vkUpdateDescriptorSets(device_, count, writes.data(), 0, nullptr);
    return true;
}

bool VulkanRenderer::upload_scene(const Scene& scene) {
    error_.clear();
    try {
        return upload_scene_impl(scene);
    } catch (const std::bad_alloc&) {
        error_ = "scene upload allocation failed";
    } catch (const std::length_error&) {
        error_ = "scene upload data exceeds a platform limit";
    } catch (const std::exception& exception) {
        error_ = std::string("scene upload failed: ") + exception.what();
    }
    return false;
}

bool VulkanRenderer::upload_scene_impl(const Scene& scene) {
    if (device_ == VK_NULL_HANDLE || command_pool_ == VK_NULL_HANDLE) {
        error_ = "cannot upload a scene before Vulkan initialization";
        return false;
    }
    if (scene.meshes().empty()) {
        error_ = "scene has no meshes to upload";
        return false;
    }
    if (scene.textures().empty()) {
        error_ = "scene has no textures to upload";
        return false;
    }
    for (const TextureData& texture : scene.textures()) {
        const std::uint64_t pixel_count =
            static_cast<std::uint64_t>(texture.width) * texture.height;
        if (texture.width == 0 || texture.height == 0 ||
            pixel_count > std::numeric_limits<std::size_t>::max() / 4u ||
            texture.rgba.size() != pixel_count * 4u) {
            error_ = "scene texture dimensions do not match its RGBA data";
            return false;
        }
    }
    for (const DrawItem& draw : scene.draws()) {
        if (draw.mesh >= scene.meshes().size() || draw.texture >= scene.textures().size() ||
            (draw.environment_texture != no_environment_texture &&
             draw.environment_texture >= scene.textures().size()) ||
            draw.transform >= scene.transforms().size()) {
            error_ = "scene draw references an invalid mesh, texture, or transform";
            return false;
        }
    }
    std::vector<GpuMesh> new_meshes(scene.meshes().size());
    std::vector<Vertex> vertices;
    std::vector<std::uint32_t> indices;
    for (std::size_t i = 0; i < scene.meshes().size(); ++i) {
        const MeshData& source = scene.meshes()[i];
        if (source.vertices.empty() || source.indices.empty() ||
            source.indices.size() > std::numeric_limits<std::uint32_t>::max()) {
            error_ = "scene mesh has no geometry or too many indices";
            return false;
        }
        for (std::uint32_t index : source.indices) {
            if (index >= source.vertices.size()) {
                error_ = "scene mesh index is outside its vertex array";
                return false;
            }
        }
        Vec3 minimum{std::numeric_limits<float>::infinity(),
                     std::numeric_limits<float>::infinity(),
                     std::numeric_limits<float>::infinity()};
        Vec3 maximum{-std::numeric_limits<float>::infinity(),
                     -std::numeric_limits<float>::infinity(),
                     -std::numeric_limits<float>::infinity()};
        bool has_transparency = false;
        for (const Vertex& vertex : source.vertices) {
            if (!std::isfinite(vertex.position.x) || !std::isfinite(vertex.position.y) ||
                !std::isfinite(vertex.position.z) || !std::isfinite(vertex.color.x) ||
                !std::isfinite(vertex.color.y) || !std::isfinite(vertex.color.z) ||
                !std::isfinite(vertex.uv.x) || !std::isfinite(vertex.uv.y) ||
                !std::isfinite(vertex.normal.x) || !std::isfinite(vertex.normal.y) ||
                !std::isfinite(vertex.normal.z) || !std::isfinite(vertex.alpha) ||
                !std::isfinite(vertex.environment_uv.x) ||
                !std::isfinite(vertex.environment_uv.y) ||
                !std::isfinite(vertex.environment_color.x) ||
                !std::isfinite(vertex.environment_color.y) ||
                !std::isfinite(vertex.environment_color.z) ||
                !std::isfinite(vertex.environment_alpha) ||
                !std::isfinite(vertex.light_response) ||
                vertex.alpha < 0.0f || vertex.alpha > 1.0f ||
                vertex.environment_alpha < 0.0f || vertex.environment_alpha > 1.0f ||
                vertex.light_response < 0.0f || vertex.light_response > 1.0f) {
                error_ = "scene mesh contains an invalid or non-finite vertex attribute";
                return false;
            }
            has_transparency |= vertex.alpha < 1.0f;
            minimum.x = std::min(minimum.x, vertex.position.x);
            minimum.y = std::min(minimum.y, vertex.position.y);
            minimum.z = std::min(minimum.z, vertex.position.z);
            maximum.x = std::max(maximum.x, vertex.position.x);
            maximum.y = std::max(maximum.y, vertex.position.y);
            maximum.z = std::max(maximum.z, vertex.position.z);
        }
        if (source.vertices.size() > std::numeric_limits<VkDeviceSize>::max() / sizeof(Vertex) ||
            source.indices.size() > std::numeric_limits<VkDeviceSize>::max() / sizeof(std::uint32_t)) {
            error_ = "scene mesh byte size overflows Vulkan buffer limits";
            return false;
        }
        if (vertices.size() > std::numeric_limits<VkDeviceSize>::max() / sizeof(Vertex) -
                                  source.vertices.size() ||
            source.indices.size() > std::numeric_limits<std::uint32_t>::max() - indices.size()) {
            error_ = "combined scene geometry exceeds Vulkan draw limits";
            return false;
        }
        GpuMesh& target = new_meshes[i];
        target.dynamic = scene.dynamic_meshes()[i];
        target.vertex_offset = vertices.size() * sizeof(Vertex);
        target.first_index = static_cast<std::uint32_t>(indices.size());
        target.index_count = static_cast<std::uint32_t>(source.indices.size());
        target.has_transparency = has_transparency;
        target.local_center = {
            static_cast<float>((static_cast<double>(minimum.x) + maximum.x) * 0.5),
            static_cast<float>((static_cast<double>(minimum.y) + maximum.y) * 0.5),
            static_cast<float>((static_cast<double>(minimum.z) + maximum.z) * 0.5)};
        vertices.insert(vertices.end(), source.vertices.begin(), source.vertices.end());
        indices.insert(indices.end(), source.indices.begin(), source.indices.end());
        if (target.dynamic)
            target.dynamic_vertex_bytes = source.vertices.size() * sizeof(Vertex);
    }

    const VkDeviceSize vertex_bytes = vertices.size() * sizeof(Vertex);
    const VkDeviceSize index_bytes = indices.size() * sizeof(std::uint32_t);
    if (vertex_bytes > std::numeric_limits<VkDeviceSize>::max() - 3u) {
        error_ = "combined scene staging alignment overflows Vulkan buffer limits";
        return false;
    }
    const VkDeviceSize index_staging_offset = (vertex_bytes + 3u) & ~VkDeviceSize{3u};
    if (index_bytes > std::numeric_limits<VkDeviceSize>::max() - index_staging_offset) {
        error_ = "combined scene staging size overflows Vulkan buffer limits";
        return false;
    }
    const VkDeviceSize staging_bytes = index_staging_offset + index_bytes;

    // Build the replacement in isolated handles. The current scene remains
    // available until all uploads and descriptors have succeeded.
    VkBuffer new_vertex_buffer = VK_NULL_HANDLE;
    VkDeviceMemory new_vertex_memory = VK_NULL_HANDLE;
    VkBuffer new_index_buffer = VK_NULL_HANDLE;
    VkDeviceMemory new_index_memory = VK_NULL_HANDLE;
    std::vector<GpuTexture> new_textures(scene.textures().size());
    VkDescriptorPool new_descriptor_pool = VK_NULL_HANDLE;
    auto destroy_staged = [this, &new_vertex_buffer, &new_vertex_memory,
                           &new_index_buffer, &new_index_memory, &new_textures,
                           &new_descriptor_pool, &new_meshes]() {
        if (new_descriptor_pool) vkDestroyDescriptorPool(device_, new_descriptor_pool, nullptr);
        new_descriptor_pool = VK_NULL_HANDLE;
        for (GpuTexture& texture : new_textures) destroy_texture(texture);
        for (GpuMesh& mesh : new_meshes) {
            for (std::size_t i = 0; i < frames_in_flight; ++i) {
                if (mesh.dynamic_vertex_mappings[i] && mesh.dynamic_vertex_memories[i])
                    vkUnmapMemory(device_, mesh.dynamic_vertex_memories[i]);
                if (mesh.dynamic_vertex_buffers[i])
                    vkDestroyBuffer(device_, mesh.dynamic_vertex_buffers[i], nullptr);
                if (mesh.dynamic_vertex_memories[i])
                    vkFreeMemory(device_, mesh.dynamic_vertex_memories[i], nullptr);
            }
            mesh.dynamic_vertex_mappings.fill(nullptr);
            mesh.dynamic_vertex_buffers.fill(VK_NULL_HANDLE);
            mesh.dynamic_vertex_memories.fill(VK_NULL_HANDLE);
        }
        if (new_vertex_buffer) vkDestroyBuffer(device_, new_vertex_buffer, nullptr);
        if (new_vertex_memory) vkFreeMemory(device_, new_vertex_memory, nullptr);
        if (new_index_buffer) vkDestroyBuffer(device_, new_index_buffer, nullptr);
        if (new_index_memory) vkFreeMemory(device_, new_index_memory, nullptr);
        new_vertex_buffer = VK_NULL_HANDLE;
        new_vertex_memory = VK_NULL_HANDLE;
        new_index_buffer = VK_NULL_HANDLE;
        new_index_memory = VK_NULL_HANDLE;
    };
    ScopeExit staged_cleanup(destroy_staged);

    for (std::size_t mesh_index = 0; mesh_index < new_meshes.size(); ++mesh_index) {
        GpuMesh& mesh = new_meshes[mesh_index];
        if (!mesh.dynamic) continue;
        for (std::size_t frame_slot = 0; frame_slot < frames_in_flight; ++frame_slot) {
            VkMemoryPropertyFlags selected_properties = 0;
            if (!create_buffer(mesh.dynamic_vertex_bytes, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                               VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT,
                               mesh.dynamic_vertex_buffers[frame_slot],
                               mesh.dynamic_vertex_memories[frame_slot],
                               &selected_properties)) {
                destroy_staged();
                return false;
            }
            mesh.dynamic_vertex_coherent[frame_slot] =
                (selected_properties & VK_MEMORY_PROPERTY_HOST_COHERENT_BIT) != 0;
            VkResult map_result = vkMapMemory(device_,
                mesh.dynamic_vertex_memories[frame_slot], 0, VK_WHOLE_SIZE, 0,
                &mesh.dynamic_vertex_mappings[frame_slot]);
            if (map_result != VK_SUCCESS) {
                destroy_staged();
                return fail("vkMapMemory(dynamic mesh)", map_result);
            }
            std::memcpy(mesh.dynamic_vertex_mappings[frame_slot],
                        scene.meshes()[mesh_index].vertices.data(),
                        static_cast<std::size_t>(mesh.dynamic_vertex_bytes));
            if (!mesh.dynamic_vertex_coherent[frame_slot]) {
                auto range = vk_struct<VkMappedMemoryRange>(
                    VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE);
                range.memory = mesh.dynamic_vertex_memories[frame_slot];
                range.offset = 0;
                range.size = VK_WHOLE_SIZE;
                const VkResult flush_result = vkFlushMappedMemoryRanges(
                    device_, 1, &range);
                if (flush_result != VK_SUCCESS) {
                    destroy_staged();
                    return fail("vkFlushMappedMemoryRanges(dynamic mesh)", flush_result);
                }
            }
        }
    }

    if (!create_buffer(vertex_bytes, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT |
                                      VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                       VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, new_vertex_buffer, new_vertex_memory) ||
        !create_buffer(index_bytes, VK_BUFFER_USAGE_INDEX_BUFFER_BIT |
                                     VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                       VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, new_index_buffer, new_index_memory)) {
        destroy_staged();
        return false;
    }

    VkBuffer staging_buffer = VK_NULL_HANDLE;
    VkDeviceMemory staging_memory = VK_NULL_HANDLE;
    VkMemoryPropertyFlags staging_properties = 0;
    auto cleanup_staging = [this, &staging_buffer, &staging_memory]() {
        if (staging_buffer) vkDestroyBuffer(device_, staging_buffer, nullptr);
        if (staging_memory) vkFreeMemory(device_, staging_memory, nullptr);
    };
    if (!create_buffer(staging_bytes, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                       VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT, staging_buffer, staging_memory,
                       &staging_properties)) {
        cleanup_staging();
        destroy_staged();
        return false;
    }
    void* mapped = nullptr;
    VkResult result = vkMapMemory(device_, staging_memory, 0, VK_WHOLE_SIZE, 0, &mapped);
    if (result != VK_SUCCESS) {
        cleanup_staging();
        destroy_staged();
        return fail("vkMapMemory(staging)", result);
    }
    std::memcpy(mapped, vertices.data(), static_cast<std::size_t>(vertex_bytes));
    std::memcpy(static_cast<std::uint8_t*>(mapped) + index_staging_offset,
                indices.data(), static_cast<std::size_t>(index_bytes));
    if ((staging_properties & VK_MEMORY_PROPERTY_HOST_COHERENT_BIT) == 0) {
        auto range = vk_struct<VkMappedMemoryRange>(VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE);
        range.memory = staging_memory;
        range.offset = 0;
        range.size = VK_WHOLE_SIZE;
        result = vkFlushMappedMemoryRanges(device_, 1, &range);
        if (result != VK_SUCCESS) {
            vkUnmapMemory(device_, staging_memory);
            cleanup_staging();
            destroy_staged();
            return fail("vkFlushMappedMemoryRanges(mesh)", result);
        }
    }
    vkUnmapMemory(device_, staging_memory);

    VkCommandBuffer upload = VK_NULL_HANDLE;
    auto alloc = vk_struct<VkCommandBufferAllocateInfo>(VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO);
    alloc.commandPool = command_pool_;
    alloc.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    alloc.commandBufferCount = 1;
    result = vkAllocateCommandBuffers(device_, &alloc, &upload);
    if (result != VK_SUCCESS) {
        cleanup_staging();
        destroy_staged();
        return fail("vkAllocateCommandBuffers(upload)", result);
    }
    auto free_upload = [this, &upload]() {
        if (upload) vkFreeCommandBuffers(device_, command_pool_, 1, &upload);
    };
    auto begin = vk_struct<VkCommandBufferBeginInfo>(VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO);
    begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    result = vkBeginCommandBuffer(upload, &begin);
    if (result != VK_SUCCESS) {
        free_upload();
        cleanup_staging();
        destroy_staged();
        return fail("vkBeginCommandBuffer(upload)", result);
    }
    VkBufferCopy vertex_copy{0, 0, vertex_bytes};
    VkBufferCopy index_copy{index_staging_offset, 0, index_bytes};
    vkCmdCopyBuffer(upload, staging_buffer, new_vertex_buffer, 1, &vertex_copy);
    vkCmdCopyBuffer(upload, staging_buffer, new_index_buffer, 1, &index_copy);
    std::array<VkBufferMemoryBarrier2, 2> barriers{};
    barriers[0] = vk_struct<VkBufferMemoryBarrier2>(VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER_2);
    barriers[0].srcStageMask = VK_PIPELINE_STAGE_2_COPY_BIT;
    barriers[0].srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
    barriers[0].dstStageMask = VK_PIPELINE_STAGE_2_VERTEX_INPUT_BIT;
    barriers[0].dstAccessMask = VK_ACCESS_2_VERTEX_ATTRIBUTE_READ_BIT;
    barriers[0].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barriers[0].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barriers[0].buffer = new_vertex_buffer;
    barriers[0].size = vertex_bytes;
    barriers[1] = vk_struct<VkBufferMemoryBarrier2>(VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER_2);
    barriers[1].srcStageMask = VK_PIPELINE_STAGE_2_COPY_BIT;
    barriers[1].srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
    barriers[1].dstStageMask = VK_PIPELINE_STAGE_2_INDEX_INPUT_BIT;
    barriers[1].dstAccessMask = VK_ACCESS_2_INDEX_READ_BIT;
    barriers[1].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barriers[1].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barriers[1].buffer = new_index_buffer;
    barriers[1].size = index_bytes;
    auto dependency = vk_struct<VkDependencyInfo>(VK_STRUCTURE_TYPE_DEPENDENCY_INFO);
    dependency.bufferMemoryBarrierCount = static_cast<std::uint32_t>(barriers.size());
    dependency.pBufferMemoryBarriers = barriers.data();
    vkCmdPipelineBarrier2(upload, &dependency);
    result = vkEndCommandBuffer(upload);
    if (result != VK_SUCCESS) {
        free_upload();
        cleanup_staging();
        destroy_staged();
        return fail("vkEndCommandBuffer(upload)", result);
    }
    auto fence_info = vk_struct<VkFenceCreateInfo>(VK_STRUCTURE_TYPE_FENCE_CREATE_INFO);
    VkFence fence = VK_NULL_HANDLE;
    result = vkCreateFence(device_, &fence_info, nullptr, &fence);
    if (result != VK_SUCCESS) {
        free_upload();
        cleanup_staging();
        destroy_staged();
        return fail("vkCreateFence(upload)", result);
    }
    VkSubmitInfo submit{};
    submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submit.commandBufferCount = 1;
    submit.pCommandBuffers = &upload;
    result = vkQueueSubmit(graphics_queue_, 1, &submit, fence);
    if (result == VK_SUCCESS) result = vkWaitForFences(device_, 1, &fence, VK_TRUE, UINT64_MAX);
    vkDestroyFence(device_, fence, nullptr);
    free_upload();
    cleanup_staging();
    if (result != VK_SUCCESS) {
        destroy_staged();
        return fail("vkQueueSubmit(mesh upload)", result);
    }
    for (std::size_t i = 0; i < scene.textures().size(); ++i) {
        if (!upload_texture(scene.textures()[i], new_textures[i])) {
            destroy_staged();
            return false;
        }
    }
    if (!create_texture_descriptors(new_textures, new_descriptor_pool)) {
        destroy_staged();
        return false;
    }
    const VkResult idle = vkDeviceWaitIdle(device_);
    if (idle != VK_SUCCESS) {
        destroy_staged();
        return fail("vkDeviceWaitIdle(scene replacement)", idle);
    }
    destroy_meshes();
    vertex_buffer_ = new_vertex_buffer;
    vertex_memory_ = new_vertex_memory;
    index_buffer_ = new_index_buffer;
    index_memory_ = new_index_memory;
    meshes_ = std::move(new_meshes);
    textures_ = std::move(new_textures);
    texture_descriptor_pool_ = new_descriptor_pool;
    new_vertex_buffer = VK_NULL_HANDLE;
    new_vertex_memory = VK_NULL_HANDLE;
    new_index_buffer = VK_NULL_HANDLE;
    new_index_memory = VK_NULL_HANDLE;
    new_descriptor_pool = VK_NULL_HANDLE;
    uploaded_asset_revision_ = scene.asset_revision();
    staged_cleanup.release();
    return true;
}

bool VulkanRenderer::draw_frame(const Mat4& view_projection, const Scene& scene) {
    error_.clear();
    if (device_ == VK_NULL_HANDLE || swapchain_ == VK_NULL_HANDLE) {
        error_ = "cannot draw a frame before Vulkan initialization";
        return false;
    }
    if (frame_submission_failed_) {
        error_ = "renderer cannot continue after a Vulkan frame synchronization failure";
        return false;
    }
    if (vertex_buffer_ == VK_NULL_HANDLE || index_buffer_ == VK_NULL_HANDLE ||
        textures_.empty()) {
        error_ = "cannot draw a frame before uploading a scene";
        return false;
    }
    if (!finite_matrix(view_projection)) {
        error_ = "view-projection matrix contains a non-finite value";
        return false;
    }
    const SceneLighting& lighting = scene.lighting();
    if (!std::isfinite(lighting.direction.x) || !std::isfinite(lighting.direction.y) ||
        !std::isfinite(lighting.direction.z) || !std::isfinite(lighting.ambient) ||
        lighting.ambient < 0.0f || lighting.ambient > 1.0f ||
        lighting.direction.x * lighting.direction.x +
            lighting.direction.y * lighting.direction.y +
            lighting.direction.z * lighting.direction.z <= 1.0e-12f) {
        error_ = "scene lighting contains an invalid direction or ambient value";
        return false;
    }
    if (scene.meshes().size() != meshes_.size() ||
        scene.textures().size() != textures_.size() ||
        scene.asset_revision() != uploaded_asset_revision_) {
        error_ = "scene assets changed after upload_scene; upload the scene again";
        return false;
    }
    for (std::size_t mesh_index = 0; mesh_index < meshes_.size(); ++mesh_index) {
        GpuMesh& gpu_mesh = meshes_[mesh_index];
        if (!gpu_mesh.dynamic) continue;
        const std::vector<Vertex>& vertices = scene.meshes()[mesh_index].vertices;
        if (vertices.size() * sizeof(Vertex) != gpu_mesh.dynamic_vertex_bytes ||
            vertices.empty()) {
            error_ = "dynamic mesh vertex layout changed after upload_scene";
            return false;
        }
        Vec3 minimum{std::numeric_limits<float>::infinity(),
                     std::numeric_limits<float>::infinity(),
                     std::numeric_limits<float>::infinity()};
        Vec3 maximum{-std::numeric_limits<float>::infinity(),
                     -std::numeric_limits<float>::infinity(),
                     -std::numeric_limits<float>::infinity()};
        gpu_mesh.has_transparency = false;
        for (const Vertex& vertex : vertices) {
            if (!std::isfinite(vertex.position.x) || !std::isfinite(vertex.position.y) ||
                !std::isfinite(vertex.position.z) || !std::isfinite(vertex.color.x) ||
                !std::isfinite(vertex.color.y) || !std::isfinite(vertex.color.z) ||
                !std::isfinite(vertex.uv.x) || !std::isfinite(vertex.uv.y) ||
                !std::isfinite(vertex.normal.x) || !std::isfinite(vertex.normal.y) ||
                !std::isfinite(vertex.normal.z) || !std::isfinite(vertex.alpha) ||
                !std::isfinite(vertex.light_response) ||
                !std::isfinite(vertex.environment_uv.x) ||
                !std::isfinite(vertex.environment_uv.y) ||
                !std::isfinite(vertex.environment_color.x) ||
                !std::isfinite(vertex.environment_color.y) ||
                !std::isfinite(vertex.environment_color.z) ||
                !std::isfinite(vertex.environment_alpha) ||
                vertex.alpha < 0.0f || vertex.alpha > 1.0f ||
                vertex.environment_alpha < 0.0f || vertex.environment_alpha > 1.0f ||
                vertex.light_response < 0.0f || vertex.light_response > 1.0f) {
                error_ = "dynamic mesh contains an invalid or non-finite vertex attribute";
                return false;
            }
            gpu_mesh.has_transparency |= vertex.alpha < 1.0f;
            minimum.x = std::min(minimum.x, vertex.position.x);
            minimum.y = std::min(minimum.y, vertex.position.y);
            minimum.z = std::min(minimum.z, vertex.position.z);
            maximum.x = std::max(maximum.x, vertex.position.x);
            maximum.y = std::max(maximum.y, vertex.position.y);
            maximum.z = std::max(maximum.z, vertex.position.z);
        }
        gpu_mesh.local_center = {
            static_cast<float>((static_cast<double>(minimum.x) + maximum.x) * 0.5),
            static_cast<float>((static_cast<double>(minimum.y) + maximum.y) * 0.5),
            static_cast<float>((static_cast<double>(minimum.z) + maximum.z) * 0.5)};
    }
    for (const DrawItem& draw : scene.draws()) {
        if (draw.mesh >= meshes_.size() || draw.texture >= textures_.size() ||
            (draw.environment_texture != no_environment_texture &&
             draw.environment_texture >= textures_.size()) ||
            draw.transform >= scene.transforms().size()) {
            error_ = "scene draw references an unavailable mesh, texture, or transform";
            return false;
        }
    }
    std::vector<Mat4> world_transforms;
    try {
        world_transforms = scene.resolved_transforms();
    } catch (const std::exception& exception) {
        error_ = std::string("could not resolve scene transforms: ") + exception.what();
        return false;
    }
    if (!std::all_of(world_transforms.begin(), world_transforms.end(), finite_matrix)) {
        error_ = "scene transform contains a non-finite value";
        return false;
    }
    std::vector<DrawSubmission> opaque_draws;
    std::vector<DrawSubmission> transparent_draws;
    opaque_draws.reserve(scene.draws().size());
    transparent_draws.reserve(scene.draws().size());
    for (const DrawItem& draw : scene.draws()) {
        if (!draw.visible) continue;
        const Mat4 world = Mat4::multiply(world_transforms[draw.transform], draw.model);
        const Mat4 view_projection_model = Mat4::multiply(view_projection, world);
        if (!finite_matrix(world) || !finite_matrix(view_projection_model)) {
            error_ = "scene draw transform produces a non-finite matrix";
            return false;
        }
        const GpuMesh& mesh = meshes_[draw.mesh];
        const Vec3 center{
            world.value[0] * mesh.local_center.x + world.value[4] * mesh.local_center.y +
                world.value[8] * mesh.local_center.z + world.value[12],
            world.value[1] * mesh.local_center.x + world.value[5] * mesh.local_center.y +
                world.value[9] * mesh.local_center.z + world.value[13],
            world.value[2] * mesh.local_center.x + world.value[6] * mesh.local_center.y +
                world.value[10] * mesh.local_center.z + world.value[14]};
        const bool transparent = textures_[draw.texture].has_transparency || mesh.has_transparency;
        DrawSubmission submission{&draw, world, view_projection_model,
                                  projected_depth(view_projection, center), transparent};
        if (transparent)
            transparent_draws.push_back(submission);
        else
            opaque_draws.push_back(submission);
    }
    std::stable_sort(transparent_draws.begin(), transparent_draws.end(),
        [](const DrawSubmission& left, const DrawSubmission& right) {
            return left.depth > right.depth;
        });
    if (resize_requested_) {
        int width = 0, height = 0;
        SDL_GetWindowSizeInPixels(window_, &width, &height);
        if (width == 0 || height == 0) return true;
        if (!recreate_swapchain()) return false;
    }
    Frame& frame = frames_[frame_index_];
    VkResult result = vkWaitForFences(device_, 1, &frame.in_flight, VK_TRUE, UINT64_MAX);
    if (result != VK_SUCCESS) return fail("vkWaitForFences", result);

    for (std::size_t mesh_index = 0; mesh_index < meshes_.size(); ++mesh_index) {
        GpuMesh& mesh = meshes_[mesh_index];
        if (!mesh.dynamic) continue;
        const std::span<const Vertex> vertices = scene.dynamic_vertices(
            static_cast<MeshId>(mesh_index));
        if (vertices.size_bytes() != mesh.dynamic_vertex_bytes) {
            error_ = "dynamic mesh vertex count changed after upload_scene";
            return false;
        }
        void* mapping = mesh.dynamic_vertex_mappings[frame_index_];
        if (!mapping) {
            error_ = "dynamic mesh has no mapped vertex storage for this frame";
            return false;
        }
        std::memcpy(mapping, vertices.data(), vertices.size_bytes());
        if (!mesh.dynamic_vertex_coherent[frame_index_]) {
            auto range = vk_struct<VkMappedMemoryRange>(
                VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE);
            range.memory = mesh.dynamic_vertex_memories[frame_index_];
            range.offset = 0;
            range.size = VK_WHOLE_SIZE;
            result = vkFlushMappedMemoryRanges(device_, 1, &range);
            if (result != VK_SUCCESS)
                return fail("vkFlushMappedMemoryRanges(dynamic frame)", result);
        }
    }

    std::uint32_t image_index = 0;
    result = vkAcquireNextImageKHR(device_, swapchain_, UINT64_MAX,
                                   frame.image_available, VK_NULL_HANDLE, &image_index);
    if (result == VK_ERROR_OUT_OF_DATE_KHR) return recreate_swapchain();
    if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR)
        return fail("vkAcquireNextImageKHR", result);
    const bool acquired_suboptimal = result == VK_SUBOPTIMAL_KHR;
    if (image_fences_[image_index] != VK_NULL_HANDLE) {
        result = vkWaitForFences(device_, 1, &image_fences_[image_index], VK_TRUE, UINT64_MAX);
        if (result != VK_SUCCESS) return fail("vkWaitForFences(image)", result);
    }
    image_fences_[image_index] = frame.in_flight;

    VkCommandBuffer command = command_buffers_[image_index];
    result = vkResetCommandBuffer(command, 0);
    if (result != VK_SUCCESS) return fail("vkResetCommandBuffer", result);
    auto begin = vk_struct<VkCommandBufferBeginInfo>(VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO);
    begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    result = vkBeginCommandBuffer(command, &begin);
    if (result != VK_SUCCESS) return fail("vkBeginCommandBuffer", result);

    auto to_color = vk_struct<VkImageMemoryBarrier2>(VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2);
    to_color.srcStageMask = VK_PIPELINE_STAGE_2_NONE;
    to_color.srcAccessMask = VK_ACCESS_2_NONE;
    to_color.dstStageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
    to_color.dstAccessMask = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT;
    to_color.oldLayout = image_initialized_[image_index] ? VK_IMAGE_LAYOUT_PRESENT_SRC_KHR : VK_IMAGE_LAYOUT_UNDEFINED;
    to_color.newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    to_color.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    to_color.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    to_color.image = images_[image_index];
    to_color.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
    auto to_depth = vk_struct<VkImageMemoryBarrier2>(VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2);
    to_depth.srcStageMask = VK_PIPELINE_STAGE_2_NONE;
    to_depth.srcAccessMask = VK_ACCESS_2_NONE;
    to_depth.dstStageMask = VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT |
                           VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT;
    to_depth.dstAccessMask = VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_READ_BIT |
                             VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    to_depth.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    to_depth.newLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
    to_depth.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    to_depth.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    to_depth.image = depth_images_[image_index];
    to_depth.subresourceRange = {VK_IMAGE_ASPECT_DEPTH_BIT, 0, 1, 0, 1};
    const std::array<VkImageMemoryBarrier2, 2> begin_barriers{to_color, to_depth};
    auto dependency = vk_struct<VkDependencyInfo>(VK_STRUCTURE_TYPE_DEPENDENCY_INFO);
    dependency.imageMemoryBarrierCount = static_cast<std::uint32_t>(begin_barriers.size());
    dependency.pImageMemoryBarriers = begin_barriers.data();
    vkCmdPipelineBarrier2(command, &dependency);

    VkClearValue clear{};
    clear.color = {{0.018f, 0.035f, 0.075f, 1.0f}};
    auto color = vk_struct<VkRenderingAttachmentInfo>(VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO);
    color.imageView = image_views_[image_index];
    color.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    color.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    color.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    color.clearValue = clear;
    auto depth_attachment = vk_struct<VkRenderingAttachmentInfo>(
        VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO);
    depth_attachment.imageView = depth_views_[image_index];
    depth_attachment.imageLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
    depth_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    depth_attachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depth_attachment.clearValue.depthStencil = {1.0f, 0};
    auto rendering = vk_struct<VkRenderingInfo>(VK_STRUCTURE_TYPE_RENDERING_INFO);
    rendering.renderArea = {{0, 0}, swapchain_extent_};
    rendering.layerCount = 1;
    rendering.colorAttachmentCount = 1;
    rendering.pColorAttachments = &color;
    rendering.pDepthAttachment = &depth_attachment;
    vkCmdBeginRendering(command, &rendering);
    VkViewport viewport{0.0f, 0.0f, static_cast<float>(swapchain_extent_.width),
                        static_cast<float>(swapchain_extent_.height), 0.0f, 1.0f};
    VkRect2D scissor{{0, 0}, swapchain_extent_};
    vkCmdSetViewport(command, 0, 1, &viewport);
    vkCmdSetScissor(command, 0, 1, &scissor);
    const auto record_draw = [this, command, &scene](const DrawSubmission& submission,
                                             bool environment_pass) {
        const DrawItem& draw = *submission.item;
        const GpuMesh& mesh = meshes_[draw.mesh];
        if (environment_pass) {
            vkCmdBindPipeline(command, VK_PIPELINE_BIND_POINT_GRAPHICS,
                              environment_pipeline_);
        } else {
            vkCmdBindPipeline(command, VK_PIPELINE_BIND_POINT_GRAPHICS,
                              submission.transparent ? transparent_pipeline_ : pipeline_);
        }
        const SceneLighting& lighting = scene.lighting();
        const DrawPushConstants push_constants{
            submission.view_projection_model, Mat4::normal_matrix(submission.world),
            {lighting.direction.x, lighting.direction.y, lighting.direction.z,
             lighting.ambient}};
        const VkBuffer draw_vertex_buffer = mesh.dynamic
            ? mesh.dynamic_vertex_buffers[frame_index_] : vertex_buffer_;
        const VkDeviceSize offset = mesh.dynamic ? 0 : mesh.vertex_offset;
        vkCmdBindVertexBuffers(command, 0, 1, &draw_vertex_buffer, &offset);
        vkCmdBindIndexBuffer(command, index_buffer_, 0, VK_INDEX_TYPE_UINT32);
        const TextureId texture = environment_pass ? draw.environment_texture : draw.texture;
        vkCmdBindDescriptorSets(command, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                pipeline_layout_, 0, 1,
                                &textures_[texture].descriptor_set, 0, nullptr);
        vkCmdPushConstants(command, pipeline_layout_,
                           VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                           0, sizeof(push_constants), &push_constants);
        vkCmdDrawIndexed(command, mesh.index_count, 1, mesh.first_index, 0, 0);
    };
    const auto record_draws = [&record_draw](const std::vector<DrawSubmission>& draws) {
        for (const DrawSubmission& draw : draws) {
            record_draw(draw, false);
            if (draw.item->environment_texture != no_environment_texture)
                record_draw(draw, true);
        }
    };
    record_draws(opaque_draws);
    record_draws(transparent_draws);
    vkCmdEndRendering(command);

    auto to_present = vk_struct<VkImageMemoryBarrier2>(VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2);
    to_present.srcStageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
    to_present.srcAccessMask = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT;
    to_present.dstStageMask = VK_PIPELINE_STAGE_2_NONE;
    to_present.dstAccessMask = VK_ACCESS_2_NONE;
    to_present.oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    to_present.newLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
    to_present.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    to_present.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    to_present.image = images_[image_index];
    to_present.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
    dependency.pImageMemoryBarriers = &to_present;
    vkCmdPipelineBarrier2(command, &dependency);
    result = vkEndCommandBuffer(command);
    if (result != VK_SUCCESS) return fail("vkEndCommandBuffer", result);

    result = vkResetFences(device_, 1, &frame.in_flight);
    if (result != VK_SUCCESS) {
        frame_submission_failed_ = true;
        return fail("vkResetFences", result);
    }
    auto wait = vk_struct<VkSemaphoreSubmitInfo>(VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO);
    wait.semaphore = frame.image_available;
    wait.stageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
    auto cmd = vk_struct<VkCommandBufferSubmitInfo>(VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO);
    cmd.commandBuffer = command;
    auto signal = vk_struct<VkSemaphoreSubmitInfo>(VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO);
    signal.semaphore = render_finished_[image_index];
    signal.stageMask = VK_PIPELINE_STAGE_2_ALL_GRAPHICS_BIT;
    auto submit = vk_struct<VkSubmitInfo2>(VK_STRUCTURE_TYPE_SUBMIT_INFO_2);
    submit.waitSemaphoreInfoCount = 1;
    submit.pWaitSemaphoreInfos = &wait;
    submit.commandBufferInfoCount = 1;
    submit.pCommandBufferInfos = &cmd;
    submit.signalSemaphoreInfoCount = 1;
    submit.pSignalSemaphoreInfos = &signal;
    result = vkQueueSubmit2(graphics_queue_, 1, &submit, frame.in_flight);
    if (result != VK_SUCCESS) {
        // The reset fence is now unsignaled, and the acquired image semaphore
        // may remain signaled. Refuse future draws instead of waiting forever
        // on synchronization that no queue submission will complete.
        frame_submission_failed_ = true;
        return fail("vkQueueSubmit2", result);
    }
    image_initialized_[image_index] = true;

    auto present = vk_struct<VkPresentInfoKHR>(VK_STRUCTURE_TYPE_PRESENT_INFO_KHR);
    present.waitSemaphoreCount = 1;
    present.pWaitSemaphores = &render_finished_[image_index];
    present.swapchainCount = 1;
    present.pSwapchains = &swapchain_;
    present.pImageIndices = &image_index;
    result = vkQueuePresentKHR(present_queue_, &present);
    frame_index_ = (frame_index_ + 1) % frames_in_flight;
    if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR ||
        resize_requested_ || acquired_suboptimal) return recreate_swapchain();
    return result == VK_SUCCESS || fail("vkQueuePresentKHR", result);
}

bool VulkanRenderer::recreate_swapchain() {
    int width = 0, height = 0;
    SDL_GetWindowSizeInPixels(window_, &width, &height);
    if (width == 0 || height == 0) {
        resize_requested_ = true;
        return true;
    }
    VkResult result = vkDeviceWaitIdle(device_);
    if (result != VK_SUCCESS) return fail("vkDeviceWaitIdle", result);
    destroy_swapchain();
    resize_requested_ = false;
    return create_swapchain();
}

void VulkanRenderer::wait_idle() {
    if (device_ != VK_NULL_HANDLE) vkDeviceWaitIdle(device_);
}

void VulkanRenderer::destroy_swapchain() {
    if (device_ == VK_NULL_HANDLE) return;
    destroy_pipeline();
    if (!command_buffers_.empty() && command_pool_ != VK_NULL_HANDLE) {
        vkFreeCommandBuffers(device_, command_pool_,
                            static_cast<std::uint32_t>(command_buffers_.size()),
                            command_buffers_.data());
    }
    command_buffers_.clear();
    for (VkImageView view : image_views_)
        if (view != VK_NULL_HANDLE) vkDestroyImageView(device_, view, nullptr);
    image_views_.clear();
    for (VkImageView view : depth_views_)
        if (view != VK_NULL_HANDLE) vkDestroyImageView(device_, view, nullptr);
    depth_views_.clear();
    for (VkImage image : depth_images_)
        if (image != VK_NULL_HANDLE) vkDestroyImage(device_, image, nullptr);
    depth_images_.clear();
    for (VkDeviceMemory memory : depth_memories_)
        if (memory != VK_NULL_HANDLE) vkFreeMemory(device_, memory, nullptr);
    depth_memories_.clear();
    for (VkSemaphore semaphore : render_finished_)
        if (semaphore != VK_NULL_HANDLE) vkDestroySemaphore(device_, semaphore, nullptr);
    render_finished_.clear();
    images_.clear();
    image_fences_.clear();
    image_initialized_.clear();
    if (swapchain_ != VK_NULL_HANDLE) vkDestroySwapchainKHR(device_, swapchain_, nullptr);
    swapchain_ = VK_NULL_HANDLE;
}

void VulkanRenderer::destroy_pipeline() {
    if (device_ == VK_NULL_HANDLE) return;
    if (pipeline_) vkDestroyPipeline(device_, pipeline_, nullptr);
    if (transparent_pipeline_) vkDestroyPipeline(device_, transparent_pipeline_, nullptr);
    if (environment_pipeline_) vkDestroyPipeline(device_, environment_pipeline_, nullptr);
    if (pipeline_layout_) vkDestroyPipelineLayout(device_, pipeline_layout_, nullptr);
    pipeline_ = VK_NULL_HANDLE;
    transparent_pipeline_ = VK_NULL_HANDLE;
    environment_pipeline_ = VK_NULL_HANDLE;
    pipeline_layout_ = VK_NULL_HANDLE;
}

void VulkanRenderer::destroy_meshes() {
    destroy_textures();
    if (device_ != VK_NULL_HANDLE) {
        for (GpuMesh& mesh : meshes_) {
            for (std::size_t i = 0; i < frames_in_flight; ++i) {
                if (mesh.dynamic_vertex_mappings[i] && mesh.dynamic_vertex_memories[i])
                    vkUnmapMemory(device_, mesh.dynamic_vertex_memories[i]);
                if (mesh.dynamic_vertex_buffers[i])
                    vkDestroyBuffer(device_, mesh.dynamic_vertex_buffers[i], nullptr);
                if (mesh.dynamic_vertex_memories[i])
                    vkFreeMemory(device_, mesh.dynamic_vertex_memories[i], nullptr);
            }
        }
        if (vertex_buffer_) vkDestroyBuffer(device_, vertex_buffer_, nullptr);
        if (vertex_memory_) vkFreeMemory(device_, vertex_memory_, nullptr);
        if (index_buffer_) vkDestroyBuffer(device_, index_buffer_, nullptr);
        if (index_memory_) vkFreeMemory(device_, index_memory_, nullptr);
    }
    vertex_buffer_ = VK_NULL_HANDLE;
    vertex_memory_ = VK_NULL_HANDLE;
    index_buffer_ = VK_NULL_HANDLE;
    index_memory_ = VK_NULL_HANDLE;
    meshes_.clear();
}

void VulkanRenderer::destroy_textures() {
    if (device_ != VK_NULL_HANDLE && texture_descriptor_pool_)
        vkDestroyDescriptorPool(device_, texture_descriptor_pool_, nullptr);
    texture_descriptor_pool_ = VK_NULL_HANDLE;
    for (GpuTexture& texture : textures_) destroy_texture(texture);
    textures_.clear();
}

void VulkanRenderer::destroy_texture(GpuTexture& texture) {
    if (device_ != VK_NULL_HANDLE) {
        if (texture.sampler) vkDestroySampler(device_, texture.sampler, nullptr);
        if (texture.view) vkDestroyImageView(device_, texture.view, nullptr);
        if (texture.image) vkDestroyImage(device_, texture.image, nullptr);
        if (texture.memory) vkFreeMemory(device_, texture.memory, nullptr);
    }
    texture.descriptor_set = VK_NULL_HANDLE;
    texture.sampler = VK_NULL_HANDLE;
    texture.view = VK_NULL_HANDLE;
    texture.image = VK_NULL_HANDLE;
    texture.memory = VK_NULL_HANDLE;
}

void VulkanRenderer::cleanup() {
    if (device_ != VK_NULL_HANDLE) vkDeviceWaitIdle(device_);
    destroy_swapchain();
    destroy_meshes();
    if (device_ != VK_NULL_HANDLE) {
        for (auto& frame : frames_) {
            if (frame.in_flight) vkDestroyFence(device_, frame.in_flight, nullptr);
            if (frame.image_available) vkDestroySemaphore(device_, frame.image_available, nullptr);
        }
        if (texture_set_layout_)
            vkDestroyDescriptorSetLayout(device_, texture_set_layout_, nullptr);
        texture_set_layout_ = VK_NULL_HANDLE;
        if (command_pool_) vkDestroyCommandPool(device_, command_pool_, nullptr);
        vkDestroyDevice(device_, nullptr);
        device_ = VK_NULL_HANDLE;
    }
    if (surface_ && instance_) vkDestroySurfaceKHR(instance_, surface_, nullptr);
    if (instance_) vkDestroyInstance(instance_, nullptr);
    surface_ = VK_NULL_HANDLE;
    instance_ = VK_NULL_HANDLE;
}

} // namespace ac5

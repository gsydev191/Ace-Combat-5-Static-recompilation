#pragma once

#include "scene.hpp"

#include <SDL3/SDL_vulkan.h>
#include <vulkan/vulkan.h>

#include <cstdint>
#include <string>
#include <vector>

namespace ac5 {

class VulkanRenderer {
public:
    VulkanRenderer() = default;
    ~VulkanRenderer();

    VulkanRenderer(const VulkanRenderer&) = delete;
    VulkanRenderer& operator=(const VulkanRenderer&) = delete;

    bool initialize(SDL_Window* window);
    // Uploads mesh and texture data. Call again after changing any mesh or
    // texture asset in the scene.
    bool upload_scene(const Scene& scene);
    // Draw items and transforms may change between frames; their IDs must refer
    // to the mesh and texture data most recently uploaded with upload_scene().
    // Asset changes are detected and require another upload_scene() call.
    bool draw_frame(const Mat4& view_projection, const Scene& scene);
    void request_resize() { resize_requested_ = true; }
    void wait_idle();
    const std::string& error() const { return error_; }

private:
    static constexpr std::uint32_t frames_in_flight = 2;

    struct Frame {
        VkSemaphore image_available = VK_NULL_HANDLE;
        VkFence in_flight = VK_NULL_HANDLE;
    };

    struct GpuMesh {
        VkDeviceSize vertex_offset = 0;
        std::uint32_t first_index = 0;
        std::uint32_t index_count = 0;
    };

    struct GpuTexture {
        VkImage image = VK_NULL_HANDLE;
        VkDeviceMemory memory = VK_NULL_HANDLE;
        VkImageView view = VK_NULL_HANDLE;
        VkSampler sampler = VK_NULL_HANDLE;
        VkDescriptorSet descriptor_set = VK_NULL_HANDLE;
        std::uint32_t mip_levels = 1;
    };

    bool create_instance();
    bool create_surface();
    bool choose_device();
    bool create_device();
    bool create_swapchain();
    bool create_depth_resources();
    bool create_pipeline();
    bool upload_texture(const TextureData& texture, GpuTexture& gpu_texture);
    bool create_texture_descriptors(std::vector<GpuTexture>& textures,
                                   VkDescriptorPool& descriptor_pool);
    bool create_buffer(VkDeviceSize size, VkBufferUsageFlags usage,
                       VkMemoryPropertyFlags properties, VkBuffer& buffer,
                       VkDeviceMemory& memory,
                       VkMemoryPropertyFlags* selected_properties = nullptr);
    bool find_memory_type(std::uint32_t mask, VkMemoryPropertyFlags properties,
                          std::uint32_t& type_index);
    bool create_frames();
    bool recreate_swapchain();
    bool fail(const char* operation, VkResult result);
    void destroy_swapchain();
    void destroy_pipeline();
    void destroy_textures();
    void destroy_texture(GpuTexture& texture);
    void destroy_meshes();
    void cleanup();

    SDL_Window* window_ = nullptr;
    VkInstance instance_ = VK_NULL_HANDLE;
    VkSurfaceKHR surface_ = VK_NULL_HANDLE;
    VkPhysicalDevice physical_device_ = VK_NULL_HANDLE;
    VkDevice device_ = VK_NULL_HANDLE;
    bool portability_subset_supported_ = false;
    VkBool32 sampler_anisotropy_supported_ = VK_FALSE;
    float max_sampler_anisotropy_ = 1.0f;
    std::uint32_t graphics_family_ = UINT32_MAX;
    std::uint32_t present_family_ = UINT32_MAX;
    VkQueue graphics_queue_ = VK_NULL_HANDLE;
    VkQueue present_queue_ = VK_NULL_HANDLE;
    VkSwapchainKHR swapchain_ = VK_NULL_HANDLE;
    VkFormat depth_format_ = VK_FORMAT_UNDEFINED;
    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
    VkPipeline pipeline_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout texture_set_layout_ = VK_NULL_HANDLE;
    VkDescriptorPool texture_descriptor_pool_ = VK_NULL_HANDLE;
    std::vector<GpuTexture> textures_;
    VkBuffer vertex_buffer_ = VK_NULL_HANDLE;
    VkDeviceMemory vertex_memory_ = VK_NULL_HANDLE;
    VkBuffer index_buffer_ = VK_NULL_HANDLE;
    VkDeviceMemory index_memory_ = VK_NULL_HANDLE;
    std::vector<GpuMesh> meshes_;
    std::uint64_t uploaded_asset_revision_ = 0;
    VkFormat swapchain_format_ = VK_FORMAT_UNDEFINED;
    VkExtent2D swapchain_extent_{};
    std::vector<VkImage> images_;
    std::vector<VkImageView> image_views_;
    std::vector<VkImage> depth_images_;
    std::vector<VkDeviceMemory> depth_memories_;
    std::vector<VkImageView> depth_views_;
    std::vector<VkSemaphore> render_finished_;
    std::vector<VkCommandBuffer> command_buffers_;
    std::vector<VkFence> image_fences_;
    std::vector<bool> image_initialized_;
    VkCommandPool command_pool_ = VK_NULL_HANDLE;
    Frame frames_[frames_in_flight]{};
    std::uint32_t frame_index_ = 0;
    bool resize_requested_ = false;
    std::string error_;
};

} // namespace ac5

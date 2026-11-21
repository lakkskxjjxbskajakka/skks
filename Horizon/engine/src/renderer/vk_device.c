#include "vk_internal.h"

static const char* device_type_name(VkPhysicalDeviceType t) {
    switch (t) {
        case VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU: return "discrete";
        case VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU: return "integrated";
        case VK_PHYSICAL_DEVICE_TYPE_VIRTUAL_GPU: return "virtual";
        case VK_PHYSICAL_DEVICE_TYPE_CPU: return "software";
        default: return "other";
    }
}

static bool has_extension(const VkExtensionProperties* props, u32 count, const char* name) {
    for (u32 i = 0; i < count; ++i) if (strcmp(props[i].extensionName, name) == 0) return true;
    return false;
}

static VKAPI_ATTR VkBool32 VKAPI_CALL debug_callback(
    VkDebugUtilsMessageSeverityFlagBitsEXT severity,
    VkDebugUtilsMessageTypeFlagsEXT type,
    const VkDebugUtilsMessengerCallbackDataEXT* data, void* user) {
    (void)type; (void)user;
    if (severity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT)
        HR_ERROR("vk", "%s", data->pMessage);
    else if (severity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT)
        HR_WARN("vk", "%s", data->pMessage);
    return VK_FALSE;
}

static void create_instance(HrRenderer* r) {
    u32 ext_count = 0;
    VK_CHECK(vkEnumerateInstanceExtensionProperties(NULL, &ext_count, NULL));
    VkExtensionProperties* exts = (VkExtensionProperties*)hr_alloc(sizeof(VkExtensionProperties) * ext_count);
    VK_CHECK(vkEnumerateInstanceExtensionProperties(NULL, &ext_count, exts));

    const char* want[4];
    u32 n = 0;
    want[n++] = VK_KHR_SURFACE_EXTENSION_NAME;
#if defined(_WIN32)
    want[n++] = VK_KHR_WIN32_SURFACE_EXTENSION_NAME;
#else
    want[n++] = VK_KHR_XLIB_SURFACE_EXTENSION_NAME;
#endif
    bool debug_ext = has_extension(exts, ext_count, VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
    if (r->validation && debug_ext) want[n++] = VK_EXT_DEBUG_UTILS_EXTENSION_NAME;

    u32 layer_count = 0;
    vkEnumerateInstanceLayerProperties(&layer_count, NULL);
    VkLayerProperties* layers = (VkLayerProperties*)hr_alloc(sizeof(VkLayerProperties) * layer_count);
    vkEnumerateInstanceLayerProperties(&layer_count, layers);
    const char* validation_layer = "VK_LAYER_KHRONOS_validation";
    bool have_validation = false;
    for (u32 i = 0; i < layer_count; ++i)
        if (strcmp(layers[i].layerName, validation_layer) == 0) have_validation = true;
    if (r->validation && !have_validation)
        HR_WARN("vulkan", "validation requested but VK_LAYER_KHRONOS_validation is not installed");

    VkApplicationInfo app = { VK_STRUCTURE_TYPE_APPLICATION_INFO };
    app.pApplicationName = "Horizon";
    app.applicationVersion = VK_MAKE_VERSION(0, 1, 0);
    app.pEngineName = "Horizon Engine";
    app.engineVersion = VK_MAKE_VERSION(0, 1, 0);
    app.apiVersion = VK_API_VERSION_1_1;

    VkInstanceCreateInfo ci = { VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO };
    ci.pApplicationInfo = &app;
    ci.enabledExtensionCount = n;
    ci.ppEnabledExtensionNames = want;
    if (r->validation && have_validation) {
        ci.enabledLayerCount = 1;
        ci.ppEnabledLayerNames = &validation_layer;
    }
    VK_CHECK(vkCreateInstance(&ci, NULL, &r->instance));

    if (r->validation && debug_ext) {
        VkDebugUtilsMessengerCreateInfoEXT dci = { VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT };
        dci.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
                              VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
        dci.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
                          VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
                          VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
        dci.pfnUserCallback = debug_callback;
        PFN_vkCreateDebugUtilsMessengerEXT fn =
            (PFN_vkCreateDebugUtilsMessengerEXT)vkGetInstanceProcAddr(r->instance, "vkCreateDebugUtilsMessengerEXT");
        if (fn) fn(r->instance, &dci, NULL, &r->debug_messenger);
    }
    hr_free(exts);
    hr_free(layers);
}

static void create_surface(HrRenderer* r) {
#if defined(_WIN32)
    VkWin32SurfaceCreateInfoKHR ci = { VK_STRUCTURE_TYPE_WIN32_SURFACE_CREATE_INFO_KHR };
    ci.hinstance = (HINSTANCE)hr_window_native(r->window, HR_NATIVE_HINSTANCE);
    ci.hwnd = (HWND)hr_window_native(r->window, HR_NATIVE_WINDOW);
    VK_CHECK(vkCreateWin32SurfaceKHR(r->instance, &ci, NULL, &r->surface));
#else
    VkXlibSurfaceCreateInfoKHR ci = { VK_STRUCTURE_TYPE_XLIB_SURFACE_CREATE_INFO_KHR };
    ci.dpy = (Display*)hr_window_native(r->window, HR_NATIVE_DISPLAY);
    ci.window = (Window)(uintptr_t)hr_window_native(r->window, HR_NATIVE_WINDOW);
    VK_CHECK(vkCreateXlibSurfaceKHR(r->instance, &ci, NULL, &r->surface));
#endif
}

static void pick_physical_device(HrRenderer* r) {
    u32 count = 0;
    VK_CHECK(vkEnumeratePhysicalDevices(r->instance, &count, NULL));
    if (!count) HR_FATAL("vulkan", "no Vulkan physical devices found");
    VkPhysicalDevice* devs = (VkPhysicalDevice*)hr_alloc(sizeof(VkPhysicalDevice) * count);
    VK_CHECK(vkEnumeratePhysicalDevices(r->instance, &count, devs));

    VkPhysicalDevice best = VK_NULL_HANDLE;
    int best_score = -1;
    u32 best_family = 0;
    for (u32 i = 0; i < count; ++i) {
        VkPhysicalDeviceProperties p;
        vkGetPhysicalDeviceProperties(devs[i], &p);
        u32 qcount = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(devs[i], &qcount, NULL);
        VkQueueFamilyProperties* qs = (VkQueueFamilyProperties*)hr_alloc(sizeof(VkQueueFamilyProperties) * qcount);
        vkGetPhysicalDeviceQueueFamilyProperties(devs[i], &qcount, qs);
        int chosen = -1;
        for (u32 q = 0; q < qcount; ++q) {
            if (!(qs[q].queueFlags & VK_QUEUE_GRAPHICS_BIT)) continue;
            VkBool32 present = VK_FALSE;
            VK_CHECK(vkGetPhysicalDeviceSurfaceSupportKHR(devs[i], q, r->surface, &present));
            if (present) { chosen = (int)q; break; }
        }
        u32 extc = 0;
        vkEnumerateDeviceExtensionProperties(devs[i], NULL, &extc, NULL);
        VkExtensionProperties* exts = (VkExtensionProperties*)hr_alloc(sizeof(VkExtensionProperties) * extc);
        vkEnumerateDeviceExtensionProperties(devs[i], NULL, &extc, exts);
        bool has_swap = has_extension(exts, extc, VK_KHR_SWAPCHAIN_EXTENSION_NAME);
        hr_free(exts);
        hr_free(qs);
        if (chosen < 0 || !has_swap) {
            HR_DEBUG("vulkan", "skip GPU: %s (%s)", p.deviceName, device_type_name(p.deviceType));
            continue;
        }
        int score = 100;
        if (p.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) score += 1000;
        else if (p.deviceType == VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU) score += 500;
        else if (p.deviceType == VK_PHYSICAL_DEVICE_TYPE_VIRTUAL_GPU) score += 200;
        else if (p.deviceType == VK_PHYSICAL_DEVICE_TYPE_CPU) score += 20;
        if (score > best_score) { best_score = score; best = devs[i]; best_family = (u32)chosen; }
    }
    hr_free(devs);
    if (best == VK_NULL_HANDLE) HR_FATAL("vulkan", "no device with graphics+present+swapchain support");
    r->physical = best;
    r->queue_family = best_family;
    vkGetPhysicalDeviceProperties(best, &r->props);
    vkGetPhysicalDeviceMemoryProperties(best, &r->mem_props);
    HR_INFO("vulkan", "GPU: %s (%s, Vulkan %u.%u.%u)", r->props.deviceName,
            device_type_name(r->props.deviceType), VK_VERSION_MAJOR(r->props.apiVersion),
            VK_VERSION_MINOR(r->props.apiVersion), VK_VERSION_PATCH(r->props.apiVersion));
}

static void create_device(HrRenderer* r) {
    float priority = 1.0f;
    VkDeviceQueueCreateInfo qi = { VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO };
    qi.queueFamilyIndex = r->queue_family;
    qi.queueCount = 1;
    qi.pQueuePriorities = &priority;

    const char* exts[] = { VK_KHR_SWAPCHAIN_EXTENSION_NAME };
    VkPhysicalDeviceFeatures avail;
    vkGetPhysicalDeviceFeatures(r->physical, &avail);
    VkPhysicalDeviceFeatures want;
    memset(&want, 0, sizeof(want));
    want.samplerAnisotropy = avail.samplerAnisotropy;

    VkDeviceCreateInfo ci = { VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO };
    ci.queueCreateInfoCount = 1;
    ci.pQueueCreateInfos = &qi;
    ci.enabledExtensionCount = 1;
    ci.ppEnabledExtensionNames = exts;
    ci.pEnabledFeatures = &want;
    VK_CHECK(vkCreateDevice(r->physical, &ci, NULL, &r->device));
    vkGetDeviceQueue(r->device, r->queue_family, 0, &r->queue);
}

void hr_vk_destroy_swapchain(HrRenderer* r) {
    for (u32 i = 0; i < r->swap_image_count; ++i) {
        if (r->swap_views[i]) vkDestroyImageView(r->device, r->swap_views[i], NULL);
        r->swap_views[i] = VK_NULL_HANDLE;
    }
    if (r->swapchain) vkDestroySwapchainKHR(r->device, r->swapchain, NULL);
    r->swapchain = VK_NULL_HANDLE;
}

void hr_vk_create_swapchain(HrRenderer* r) {
    VkSurfaceCapabilitiesKHR caps;
    VK_CHECK(vkGetPhysicalDeviceSurfaceCapabilitiesKHR(r->physical, r->surface, &caps));

    u32 format_count = 0;
    VK_CHECK(vkGetPhysicalDeviceSurfaceFormatsKHR(r->physical, r->surface, &format_count, NULL));
    VkSurfaceFormatKHR* formats = (VkSurfaceFormatKHR*)hr_alloc(sizeof(VkSurfaceFormatKHR) * format_count);
    VK_CHECK(vkGetPhysicalDeviceSurfaceFormatsKHR(r->physical, r->surface, &format_count, formats));

    VkSurfaceFormatKHR chosen = formats[0];
    bool srgb = false;
    bool found = false;
    /* Prefer a UNORM swapchain: post.sh writes display-encoded values, so an
       sRGB format would encode a second time. */
    for (u32 i = 0; i < format_count; ++i) {
        if ((formats[i].format == VK_FORMAT_B8G8R8A8_UNORM ||
             formats[i].format == VK_FORMAT_R8G8B8A8_UNORM) &&
            formats[i].colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
            chosen = formats[i]; srgb = false; found = true; break;
        }
    }
    if (!found) {
        for (u32 i = 0; i < format_count; ++i) {
            if ((formats[i].format == VK_FORMAT_B8G8R8A8_SRGB ||
                 formats[i].format == VK_FORMAT_R8G8B8A8_SRGB) &&
                formats[i].colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
                chosen = formats[i]; srgb = true; found = true; break;
            }
        }
    }
    if (!found) {
        for (u32 i = 0; i < format_count; ++i) {
            if (formats[i].colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) { chosen = formats[i]; break; }
        }
    }
    hr_free(formats);

    VkExtent2D extent = caps.currentExtent;
    if (extent.width == 0xffffffffu) {
        int w, h;
        hr_window_get_framebuffer_size(r->window, &w, &h);
        extent.width = (u32)HR_CLAMP(w, (int)caps.minImageExtent.width, (int)caps.maxImageExtent.width);
        extent.height = (u32)HR_CLAMP(h, (int)caps.minImageExtent.height, (int)caps.maxImageExtent.height);
    }
    if (extent.width == 0 || extent.height == 0) { extent.width = 1280; extent.height = 720; }

    u32 count = caps.minImageCount + 1;
    if (caps.maxImageCount > 0 && count > caps.maxImageCount) count = caps.maxImageCount;

    VkCompositeAlphaFlagBitsKHR alpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    if (!(caps.supportedCompositeAlpha & alpha))
        alpha = VK_COMPOSITE_ALPHA_PRE_MULTIPLIED_BIT_KHR;

    VkSwapchainCreateInfoKHR ci = { VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR };
    ci.surface = r->surface;
    ci.minImageCount = count;
    ci.imageFormat = chosen.format;
    ci.imageColorSpace = chosen.colorSpace;
    ci.imageExtent = extent;
    ci.imageArrayLayers = 1;
    ci.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
    ci.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    ci.preTransform = caps.currentTransform;
    ci.compositeAlpha = alpha;
    ci.presentMode = VK_PRESENT_MODE_FIFO_KHR;

    u32 mode_count = 0;
    vkGetPhysicalDeviceSurfacePresentModesKHR(r->physical, r->surface, &mode_count, NULL);
    VkPresentModeKHR modes[8];
    if (mode_count > 8) mode_count = 8;
    vkGetPhysicalDeviceSurfacePresentModesKHR(r->physical, r->surface, &mode_count, modes);
    if (!r->vsync) {
        for (u32 i = 0; i < mode_count; ++i) {
            if (modes[i] == VK_PRESENT_MODE_MAILBOX_KHR) { ci.presentMode = modes[i]; break; }
            if (modes[i] == VK_PRESENT_MODE_IMMEDIATE_KHR) ci.presentMode = modes[i];
        }
    }
    ci.clipped = VK_TRUE;
    VK_CHECK(vkCreateSwapchainKHR(r->device, &ci, NULL, &r->swapchain));

    r->swap_format = chosen.format;
    r->swapchain_srgb = srgb;
    r->swap_extent = extent;
    VK_CHECK(vkGetSwapchainImagesKHR(r->device, r->swapchain, &r->swap_image_count, NULL));
    if (r->swap_image_count > 8) r->swap_image_count = 8;
    VK_CHECK(vkGetSwapchainImagesKHR(r->device, r->swapchain, &r->swap_image_count, r->swap_images));
    for (u32 i = 0; i < r->swap_image_count; ++i)
        r->swap_views[i] = hr_vk_create_view(r, r->swap_images[i], chosen.format,
                                             VK_IMAGE_ASPECT_COLOR_BIT, 1, 1, VK_IMAGE_VIEW_TYPE_2D);
    HR_INFO("vulkan", "swapchain %ux%u, %u images, format %d%s, mode %d",
            extent.width, extent.height, r->swap_image_count, (int)chosen.format,
            srgb ? " sRGB" : "", (int)ci.presentMode);
}

bool hr_vk_recreate_swapchain(HrRenderer* r) {
    int w, h;
    hr_window_get_framebuffer_size(r->window, &w, &h);
    if (w <= 0 || h <= 0) return false;
    VK_CHECK(vkDeviceWaitIdle(r->device));
    hr_vk_destroy_swapchain(r);
    hr_vk_create_swapchain(r);
    r->framebuffer_dirty = true;
    return true;
}

static void create_render_passes(HrRenderer* r) {
    {
        VkAttachmentDescription att[2], zero;
        memset(&zero, 0, sizeof(zero));
        att[0] = zero;
        att[0].format = VK_FORMAT_R16G16B16A16_SFLOAT;
        att[0].samples = VK_SAMPLE_COUNT_1_BIT;
        att[0].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        att[0].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        att[0].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        att[0].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        att[0].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        att[0].finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        att[1] = zero;
        att[1].format = VK_FORMAT_D32_SFLOAT;
        att[1].samples = VK_SAMPLE_COUNT_1_BIT;
        att[1].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        att[1].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        att[1].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        att[1].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        att[1].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        att[1].finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;
        VkAttachmentReference color_ref = { 0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL };
        VkAttachmentReference depth_ref = { 1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL };
        VkSubpassDescription sub;
        memset(&sub, 0, sizeof(sub));
        sub.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        sub.colorAttachmentCount = 1;
        sub.pColorAttachments = &color_ref;
        sub.pDepthStencilAttachment = &depth_ref;
        VkRenderPassCreateInfo ci = { VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO };
        ci.attachmentCount = 2;
        ci.pAttachments = att;
        ci.subpassCount = 1;
        ci.pSubpasses = &sub;
        VK_CHECK(vkCreateRenderPass(r->device, &ci, NULL, &r->hdr_rp));
    }
    {
        VkAttachmentDescription att;
        memset(&att, 0, sizeof(att));
        att.format = VK_FORMAT_D32_SFLOAT;
        att.samples = VK_SAMPLE_COUNT_1_BIT;
        att.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        att.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        att.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        att.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        att.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        att.finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        VkAttachmentReference ref = { 0, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL };
        VkSubpassDescription sub;
        memset(&sub, 0, sizeof(sub));
        sub.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        sub.pDepthStencilAttachment = &ref;
        VkRenderPassCreateInfo ci = { VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO };
        ci.attachmentCount = 1;
        ci.pAttachments = &att;
        ci.subpassCount = 1;
        ci.pSubpasses = &sub;
        VK_CHECK(vkCreateRenderPass(r->device, &ci, NULL, &r->shadow_rp));
    }
    {
        VkAttachmentDescription att;
        memset(&att, 0, sizeof(att));
        att.format = VK_FORMAT_R16G16B16A16_SFLOAT;
        att.samples = VK_SAMPLE_COUNT_1_BIT;
        att.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        att.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        att.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        att.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        att.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        att.finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        VkAttachmentReference ref = { 0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL };
        VkSubpassDescription sub;
        memset(&sub, 0, sizeof(sub));
        sub.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        sub.colorAttachmentCount = 1;
        sub.pColorAttachments = &ref;
        VkRenderPassCreateInfo ci = { VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO };
        ci.attachmentCount = 1;
        ci.pAttachments = &att;
        ci.subpassCount = 1;
        ci.pSubpasses = &sub;
        VK_CHECK(vkCreateRenderPass(r->device, &ci, NULL, &r->bloom_rp));
    }
    {
        VkAttachmentDescription att;
        memset(&att, 0, sizeof(att));
        att.format = r->swap_format;
        att.samples = VK_SAMPLE_COUNT_1_BIT;
        att.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        att.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        att.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        att.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        att.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        att.finalLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
        VkAttachmentReference ref = { 0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL };
        VkSubpassDescription sub;
        memset(&sub, 0, sizeof(sub));
        sub.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        sub.colorAttachmentCount = 1;
        sub.pColorAttachments = &ref;
        VkRenderPassCreateInfo ci = { VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO };
        ci.attachmentCount = 1;
        ci.pAttachments = &att;
        ci.subpassCount = 1;
        ci.pSubpasses = &sub;
        VK_CHECK(vkCreateRenderPass(r->device, &ci, NULL, &r->ldr_rp));
    }
}

static VkDescriptorSetLayoutBinding make_binding(u32 b, VkDescriptorType type,
                                                 VkShaderStageFlags stages) {
    VkDescriptorSetLayoutBinding r;
    memset(&r, 0, sizeof(r));
    r.binding = b;
    r.descriptorType = type;
    r.descriptorCount = 1;
    r.stageFlags = stages;
    return r;
}

static void create_descriptor_layouts(HrRenderer* r) {
    {
        VkDescriptorSetLayoutBinding b[2] = {
            make_binding(0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
                         VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT),
            make_binding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT)
        };
        VkDescriptorSetLayoutCreateInfo ci = { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
        ci.bindingCount = 2;
        ci.pBindings = b;
        VK_CHECK(vkCreateDescriptorSetLayout(r->device, &ci, NULL, &r->global_dsl));
    }
    {
        VkDescriptorSetLayoutBinding b[5];
        b[0] = make_binding(0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, VK_SHADER_STAGE_FRAGMENT_BIT);
        for (u32 i = 1; i < 5; ++i)
            b[i] = make_binding(i, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT);
        VkDescriptorSetLayoutCreateInfo ci = { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
        ci.bindingCount = 5;
        ci.pBindings = b;
        VK_CHECK(vkCreateDescriptorSetLayout(r->device, &ci, NULL, &r->material_dsl));
    }
    {
        VkDescriptorSetLayoutBinding b = make_binding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                      VK_SHADER_STAGE_FRAGMENT_BIT);
        VkDescriptorSetLayoutCreateInfo ci = { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
        ci.bindingCount = 1;
        ci.pBindings = &b;
        VK_CHECK(vkCreateDescriptorSetLayout(r->device, &ci, NULL, &r->post_dsl_a));
    }
    {
        VkDescriptorSetLayoutBinding b[2] = {
            make_binding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT),
            make_binding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT)
        };
        VkDescriptorSetLayoutCreateInfo ci = { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
        ci.bindingCount = 2;
        ci.pBindings = b;
        VK_CHECK(vkCreateDescriptorSetLayout(r->device, &ci, NULL, &r->post_dsl_b));
    }

    VkDescriptorPoolSize sizes[2] = {
        { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 16 },
        { VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 64 }
    };
    VkDescriptorPoolCreateInfo pi = { VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
    pi.maxSets = 32;
    pi.poolSizeCount = 2;
    pi.pPoolSizes = sizes;
    VK_CHECK(vkCreateDescriptorPool(r->device, &pi, NULL, &r->pool_frame));

    VkDescriptorPoolSize msizes[2] = {
        { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 32 },
        { VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 128 }
    };
    VkDescriptorPoolCreateInfo mpi = { VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
    mpi.maxSets = 32;
    mpi.poolSizeCount = 2;
    mpi.pPoolSizes = msizes;
    VK_CHECK(vkCreateDescriptorPool(r->device, &mpi, NULL, &r->pool_material));
}

static void create_pipeline_layouts(HrRenderer* r) {
    {
        VkDescriptorSetLayout sets[2] = { r->global_dsl, r->material_dsl };
        VkPipelineLayoutCreateInfo ci = { VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
        ci.setLayoutCount = 2;
        ci.pSetLayouts = sets;
        VK_CHECK(vkCreatePipelineLayout(r->device, &ci, NULL, &r->scene_layout));
    }
    {
        VkPipelineLayoutCreateInfo ci = { VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
        ci.setLayoutCount = 1;
        ci.pSetLayouts = &r->global_dsl;
        VK_CHECK(vkCreatePipelineLayout(r->device, &ci, NULL, &r->sky_layout));
        VkPushConstantRange pcr = { VK_SHADER_STAGE_VERTEX_BIT, 0, 4 };
        ci.pushConstantRangeCount = 1;
        ci.pPushConstantRanges = &pcr;
        VK_CHECK(vkCreatePipelineLayout(r->device, &ci, NULL, &r->shadow_layout));
    }
    {
        VkPushConstantRange pc = { VK_SHADER_STAGE_FRAGMENT_BIT, 0, 16 };
        VkPipelineLayoutCreateInfo ci = { VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
        ci.setLayoutCount = 1;
        ci.pSetLayouts = &r->post_dsl_a;
        ci.pushConstantRangeCount = 1;
        ci.pPushConstantRanges = &pc;
        VK_CHECK(vkCreatePipelineLayout(r->device, &ci, NULL, &r->post_layout_a));
    }
    {
        VkPushConstantRange pc = { VK_SHADER_STAGE_FRAGMENT_BIT, 0, 48 };
        VkPipelineLayoutCreateInfo ci = { VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
        ci.setLayoutCount = 1;
        ci.pSetLayouts = &r->post_dsl_b;
        ci.pushConstantRangeCount = 1;
        ci.pPushConstantRanges = &pc;
        VK_CHECK(vkCreatePipelineLayout(r->device, &ci, NULL, &r->post_layout_b));
    }
}

static void create_pipelines(HrRenderer* r) {
    VkVertexInputBindingDescription binds[2] = {
        { 0, sizeof(HrVertex), VK_VERTEX_INPUT_RATE_VERTEX },
        { 1, sizeof(HrInstanceGpu), VK_VERTEX_INPUT_RATE_INSTANCE }
    };
    VkVertexInputAttributeDescription scene_attrs[10] = {
        { 0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(HrVertex, position) },
        { 1, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(HrVertex, normal) },
        { 2, 0, VK_FORMAT_R32G32B32A32_SFLOAT, offsetof(HrVertex, tangent) },
        { 3, 0, VK_FORMAT_R32G32_SFLOAT, offsetof(HrVertex, uv) },
        { 5, 1, VK_FORMAT_R32G32B32A32_SFLOAT, offsetof(HrInstanceGpu, model) + 0 },
        { 6, 1, VK_FORMAT_R32G32B32A32_SFLOAT, offsetof(HrInstanceGpu, model) + 16 },
        { 7, 1, VK_FORMAT_R32G32B32A32_SFLOAT, offsetof(HrInstanceGpu, model) + 32 },
        { 8, 1, VK_FORMAT_R32G32B32A32_SFLOAT, offsetof(HrInstanceGpu, model) + 48 },
        { 9, 1, VK_FORMAT_R32G32B32A32_SFLOAT, offsetof(HrInstanceGpu, color) },
        { 10, 1, VK_FORMAT_R32G32B32A32_SFLOAT, offsetof(HrInstanceGpu, params) },
    };
    VkVertexInputAttributeDescription shadow_attrs[5] = {
        { 0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(HrVertex, position) },
        { 5, 1, VK_FORMAT_R32G32B32A32_SFLOAT, offsetof(HrInstanceGpu, model) + 0 },
        { 6, 1, VK_FORMAT_R32G32B32A32_SFLOAT, offsetof(HrInstanceGpu, model) + 16 },
        { 7, 1, VK_FORMAT_R32G32B32A32_SFLOAT, offsetof(HrInstanceGpu, model) + 32 },
        { 8, 1, VK_FORMAT_R32G32B32A32_SFLOAT, offsetof(HrInstanceGpu, model) + 48 },
    };

    r->scene_pipeline = hr_vk_create_pipeline(r, r->scene_layout, r->hdr_rp,
        "scene.vert.spv", "scene.frag.spv", binds, 2, scene_attrs, 10, true, true, false);
    r->sky_pipeline = hr_vk_create_pipeline(r, r->sky_layout, r->hdr_rp,
        "sky.vert.spv", "sky.frag.spv", NULL, 0, NULL, 0, false, false, false);
    r->shadow_pipeline = hr_vk_create_pipeline(r, r->shadow_layout, r->shadow_rp,
        "shadow.vert.spv", NULL, binds, 2, shadow_attrs, 5, true, true, true);
    r->bright_pipeline = hr_vk_create_pipeline(r, r->post_layout_a, r->bloom_rp,
        "fullscreen.vert.spv", "bright.frag.spv", NULL, 0, NULL, 0, false, false, false);
    r->blur_pipeline = hr_vk_create_pipeline(r, r->post_layout_a, r->bloom_rp,
        "fullscreen.vert.spv", "blur.frag.spv", NULL, 0, NULL, 0, false, false, false);
    r->post_pipeline = hr_vk_create_pipeline(r, r->post_layout_b, r->ldr_rp,
        "fullscreen.vert.spv", "post.frag.spv", NULL, 0, NULL, 0, false, false, false);
}

bool hr_renderer_init(HrRenderer** out, const HrRendererDesc* desc) {
    if (!desc || !desc->window) return false;
    HrRenderer* r = (HrRenderer*)hr_calloc(1, sizeof(HrRenderer));
    if (!r) return false;
    r->window = desc->window;
    r->validation = desc->validation;
    r->vsync = desc->vsync;
    hr_da_init(&r->draws);
    hr_da_init(&r->batches);
    hr_da_reserve(&r->draws, HR_VK_MAX_DRAWS);
    hr_da_reserve(&r->batches, 1024);
    hr_window_get_framebuffer_size(r->window, &r->window_w, &r->window_h);

    create_instance(r);
    create_surface(r);
    pick_physical_device(r);
    create_device(r);

    {
        VkSamplerCreateInfo si = { VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
        si.magFilter = VK_FILTER_NEAREST;
        si.minFilter = VK_FILTER_NEAREST;
        si.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
        si.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
        si.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
        si.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
        si.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
        si.maxLod = 0.0f;
        VK_CHECK(vkCreateSampler(r->device, &si, NULL, &r->shadow_sampler));
    }

    VkCommandPoolCreateInfo pci = { VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO };
    pci.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    pci.queueFamilyIndex = r->queue_family;
    VK_CHECK(vkCreateCommandPool(r->device, &pci, NULL, &r->cmd_pool));
    VkCommandBufferAllocateInfo cai = { VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
    cai.commandPool = r->cmd_pool;
    cai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cai.commandBufferCount = 1;
    VK_CHECK(vkAllocateCommandBuffers(r->device, &cai, &r->upload_cmd));

    hr_vk_create_swapchain(r);
    create_render_passes(r);
    create_descriptor_layouts(r);
    create_pipeline_layouts(r);
    create_pipelines(r);

    HrGpuBuffer staging[HR_TEX_PRESET_COUNT * 3];
    u32 staging_count = 0;
    VkCommandBuffer cmd = hr_vk_begin_upload(r);
    hr_vk_create_textures(r, cmd, staging, &staging_count);
    hr_vk_end_upload(r, cmd);
    for (u32 i = 0; i < staging_count; ++i) hr_vk_destroy_buffer(r, &staging[i]);
    hr_vk_create_materials(r);

    if (!hr_vk_init_frames(r)) {
        HR_ERROR("renderer", "failed to create frame resources");
        hr_renderer_shutdown(r);
        return false;
    }
    r->frame.render_scale = 1.0f;
    r->frame.shadow_resolution = desc->shadow_resolution > 0 ? desc->shadow_resolution : 2048;
    hr_window_get_framebuffer_size(r->window, &r->window_w, &r->window_h);
    *out = r;
    HR_INFO("renderer", "Horizon Vulkan renderer ready (%s)", r->props.deviceName);
    return true;
}

void hr_renderer_wait_idle(HrRenderer* r) {
    if (r && r->device) vkDeviceWaitIdle(r->device);
}

void hr_renderer_shutdown(HrRenderer* r) {
    if (!r) return;
    if (r->device) vkDeviceWaitIdle(r->device);
    if (r->device) {
        hr_vk_shutdown_frames(r);
        for (int i = 0; i < HR_TEX_PRESET_COUNT; ++i) {
            HrGpuTexture* ta = &r->tex_albedo[i];
            HrGpuTexture* tn = &r->tex_normal[i];
            HrGpuTexture* to = &r->tex_orm[i];
            if (ta->sampler) vkDestroySampler(r->device, ta->sampler, NULL);
            if (ta->view) vkDestroyImageView(r->device, ta->view, NULL);
            hr_vk_destroy_image(r, &ta->image, &ta->memory);
            if (tn->sampler) vkDestroySampler(r->device, tn->sampler, NULL);
            if (tn->view) vkDestroyImageView(r->device, tn->view, NULL);
            hr_vk_destroy_image(r, &tn->image, &tn->memory);
            if (to->sampler) vkDestroySampler(r->device, to->sampler, NULL);
            if (to->view) vkDestroyImageView(r->device, to->view, NULL);
            hr_vk_destroy_image(r, &to->image, &to->memory);
        }
        for (int i = 0; i < HR_MAT_COUNT; ++i) {
            HrGpuMaterial* m = &r->materials[i];
            if (m->ubo_mem) {
                if (m->ubo_map) vkUnmapMemory(r->device, m->ubo_mem);
                vkFreeMemory(r->device, m->ubo_mem, NULL);
            }
            if (m->ubo) vkDestroyBuffer(r->device, m->ubo, NULL);
        }
        for (int i = 0; i < r->mesh_count; ++i) {
            hr_vk_destroy_buffer(r, &r->meshes[i].vbo);
            hr_vk_destroy_buffer(r, &r->meshes[i].ibo);
        }
        if (r->post_pipeline) vkDestroyPipeline(r->device, r->post_pipeline, NULL);
        if (r->blur_pipeline) vkDestroyPipeline(r->device, r->blur_pipeline, NULL);
        if (r->bright_pipeline) vkDestroyPipeline(r->device, r->bright_pipeline, NULL);
        if (r->shadow_pipeline) vkDestroyPipeline(r->device, r->shadow_pipeline, NULL);
        if (r->scene_pipeline) vkDestroyPipeline(r->device, r->scene_pipeline, NULL);
        if (r->sky_pipeline) vkDestroyPipeline(r->device, r->sky_pipeline, NULL);
        if (r->scene_layout) vkDestroyPipelineLayout(r->device, r->scene_layout, NULL);
        if (r->sky_layout) vkDestroyPipelineLayout(r->device, r->sky_layout, NULL);
        if (r->shadow_layout) vkDestroyPipelineLayout(r->device, r->shadow_layout, NULL);
        if (r->post_layout_a) vkDestroyPipelineLayout(r->device, r->post_layout_a, NULL);
        if (r->post_layout_b) vkDestroyPipelineLayout(r->device, r->post_layout_b, NULL);
        if (r->pool_material) vkDestroyDescriptorPool(r->device, r->pool_material, NULL);
        if (r->pool_frame) vkDestroyDescriptorPool(r->device, r->pool_frame, NULL);
        if (r->global_dsl) vkDestroyDescriptorSetLayout(r->device, r->global_dsl, NULL);
        if (r->material_dsl) vkDestroyDescriptorSetLayout(r->device, r->material_dsl, NULL);
        if (r->post_dsl_a) vkDestroyDescriptorSetLayout(r->device, r->post_dsl_a, NULL);
        if (r->post_dsl_b) vkDestroyDescriptorSetLayout(r->device, r->post_dsl_b, NULL);
        if (r->shadow_sampler) vkDestroySampler(r->device, r->shadow_sampler, NULL);
        if (r->hdr_rp) vkDestroyRenderPass(r->device, r->hdr_rp, NULL);
        if (r->shadow_rp) vkDestroyRenderPass(r->device, r->shadow_rp, NULL);
        if (r->bloom_rp) vkDestroyRenderPass(r->device, r->bloom_rp, NULL);
        if (r->ldr_rp) vkDestroyRenderPass(r->device, r->ldr_rp, NULL);
        hr_vk_destroy_swapchain(r);
        if (r->cmd_pool) vkDestroyCommandPool(r->device, r->cmd_pool, NULL);
        vkDestroyDevice(r->device, NULL);
    }
    if (r->surface) vkDestroySurfaceKHR(r->instance, r->surface, NULL);
    if (r->debug_messenger) {
        PFN_vkDestroyDebugUtilsMessengerEXT fn =
            (PFN_vkDestroyDebugUtilsMessengerEXT)vkGetInstanceProcAddr(r->instance, "vkDestroyDebugUtilsMessengerEXT");
        if (fn) fn(r->instance, r->debug_messenger, NULL);
    }
    if (r->instance) vkDestroyInstance(r->instance, NULL);
    hr_da_free(&r->draws);
    hr_da_free(&r->batches);
    hr_free(r);
}

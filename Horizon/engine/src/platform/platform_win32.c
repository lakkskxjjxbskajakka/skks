#if defined(_WIN32)

#include "horizon/platform.h"
#include "horizon/memory.h"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <stdio.h>

struct HrWindow {
    HWND hwnd;
    HINSTANCE hinstance;
    int width, height;
    bool should_close;
    bool hidden;
    bool mouse_captured;
    float mouse_dx, mouse_dy, wheel;
    int last_mouse_x, last_mouse_y;
    bool have_last_mouse;
    u8 down[HR_KEY_COUNT];
    u8 prev[HR_KEY_COUNT];
    u8 pressed[HR_KEY_COUNT];
    u8 released[HR_KEY_COUNT];
    char title[256];
};

static int g_class_registered = 0;
static const wchar_t* HR_WNDCLASS = L"HorizonWindowClass";
static HrWindow* g_window_from_hwnd = NULL;

static int map_vk(WPARAM vk) {
    if (vk >= 'A' && vk <= 'Z') return HR_KEY_A + (int)(vk - 'A');
    if (vk >= '0' && vk <= '9') return HR_KEY_0 + (int)(vk - '0');
    switch (vk) {
        case VK_LEFT: return HR_KEY_LEFT;
        case VK_RIGHT: return HR_KEY_RIGHT;
        case VK_UP: return HR_KEY_UP;
        case VK_DOWN: return HR_KEY_DOWN;
        case VK_SPACE: return HR_KEY_SPACE;
        case VK_RETURN: return HR_KEY_ENTER;
        case VK_ESCAPE: return HR_KEY_ESCAPE;
        case VK_TAB: return HR_KEY_TAB;
        case VK_SHIFT: case VK_LSHIFT: case VK_RSHIFT: return HR_KEY_SHIFT;
        case VK_CONTROL: case VK_LCONTROL: case VK_RCONTROL: return HR_KEY_CTRL;
        case VK_MENU: return HR_KEY_ALT;
        case VK_F1: return HR_KEY_F1; case VK_F2: return HR_KEY_F2;
        case VK_F3: return HR_KEY_F3; case VK_F4: return HR_KEY_F4;
        case VK_F5: return HR_KEY_F5; case VK_F6: return HR_KEY_F6;
        case VK_F7: return HR_KEY_F7; case VK_F8: return HR_KEY_F8;
        case VK_F9: return HR_KEY_F9; case VK_F10: return HR_KEY_F10;
        case VK_F11: return HR_KEY_F11; case VK_F12: return HR_KEY_F12;
        default: return HR_KEY_UNKNOWN;
    }
}

static LRESULT CALLBACK wnd_proc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    HrWindow* w = g_window_from_hwnd;
    switch (msg) {
        case WM_CLOSE:
            if (w) w->should_close = true;
            return 0;
        case WM_DESTROY:
            PostQuitMessage(0);
            return 0;
        case WM_SIZE:
            if (w && wp != SIZE_MINIMIZED) {
                w->width = LOWORD(lp);
                w->height = HIWORD(lp);
            }
            return 0;
        case WM_SYSKEYDOWN:
        case WM_KEYDOWN:
            if (w) {
                int k = map_vk(wp);
                if (k >= 0 && k < HR_KEY_COUNT) w->down[k] = 1;
            }
            return 0;
        case WM_SYSKEYUP:
        case WM_KEYUP:
            if (w) {
                int k = map_vk(wp);
                if (k >= 0 && k < HR_KEY_COUNT) w->down[k] = 0;
            }
            return 0;
        case WM_MOUSEMOVE:
            if (w) {
                int mx = (int)(short)LOWORD(lp), my = (int)(short)HIWORD(lp);
                if (w->mouse_captured) {
                    int cx = w->width / 2, cy = w->height / 2;
                    w->mouse_dx += (float)(mx - cx);
                    w->mouse_dy += (float)(my - cy);
                    if (mx != cx || my != cy) SetCursorPos(cx, cy);
                } else if (w->have_last_mouse) {
                    w->mouse_dx += (float)(mx - w->last_mouse_x);
                    w->mouse_dy += (float)(my - w->last_mouse_y);
                }
                w->last_mouse_x = mx;
                w->last_mouse_y = my;
                w->have_last_mouse = true;
            }
            return 0;
        case WM_MOUSEWHEEL:
            if (w) w->wheel += (float)GET_WHEEL_DELTA_WPARAM(wp) / (float)WHEEL_DELTA;
            return 0;
        default:
            break;
    }
    return DefWindowProcW(hwnd, msg, wp, lp);
}

bool hr_platform_init(void) { return true; }
void hr_platform_shutdown(void) {}
const char* hr_platform_name(void) { return "Windows/Win32"; }

HrWindow* hr_window_create(const HrWindowDesc* desc) {
    HrWindow* w = (HrWindow*)hr_calloc(1, sizeof(HrWindow));
    if (!w) return NULL;
    w->hinstance = GetModuleHandleW(NULL);
    w->width = desc->width > 0 ? desc->width : 1280;
    w->height = desc->height > 0 ? desc->height : 720;
    w->hidden = desc->hidden;
    snprintf(w->title, sizeof(w->title), "%s", desc->title ? desc->title : "Horizon");

    if (!g_class_registered) {
        WNDCLASSEXW wc = {0};
        wc.cbSize = sizeof(wc);
        wc.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
        wc.lpfnWndProc = wnd_proc;
        wc.hInstance = w->hinstance;
        wc.hCursor = LoadCursor(NULL, IDC_ARROW);
        wc.hbrBackground = (HBRUSH)GetStockObject(BLACK_BRUSH);
        wc.lpszClassName = HR_WNDCLASS;
        if (!RegisterClassExW(&wc)) { hr_free(w); return NULL; }
        g_class_registered = 1;
    }

    wchar_t wtitle[256];
    MultiByteToWideChar(CP_UTF8, 0, w->title, -1, wtitle, 256);
    DWORD style = WS_OVERLAPPEDWINDOW;
    if (!desc->resizable) style &= ~(WS_THICKFRAME | WS_MAXIMIZEBOX);
    RECT r = {0, 0, w->width, w->height};
    AdjustWindowRect(&r, style, FALSE);
    w->hwnd = CreateWindowExW(0, HR_WNDCLASS, wtitle, style,
                              CW_USEDEFAULT, CW_USEDEFAULT,
                              r.right - r.left, r.bottom - r.top,
                              NULL, NULL, w->hinstance, NULL);
    if (!w->hwnd) { hr_free(w); return NULL; }
    g_window_from_hwnd = w;
    if (!desc->hidden) {
        ShowWindow(w->hwnd, SW_SHOW);
        SetForegroundWindow(w->hwnd);
        SetFocus(w->hwnd);
    }
    HR_INFO("platform", "window %dx%d created (Win32)", w->width, w->height);
    return w;
}

void hr_window_destroy(HrWindow* w) {
    if (!w) return;
    if (w->hwnd) DestroyWindow(w->hwnd);
    if (g_window_from_hwnd == w) g_window_from_hwnd = NULL;
    hr_free(w);
}

bool hr_window_poll(HrWindow* w) {
    w->mouse_dx = w->mouse_dy = w->wheel = 0.0f;
    MSG msg;
    while (PeekMessageW(&msg, NULL, 0, 0, PM_REMOVE)) {
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
    }
    for (int i = 0; i < HR_KEY_COUNT; ++i) {
        w->pressed[i] = (u8)(w->down[i] && !w->prev[i]);
        w->released[i] = (u8)(!w->down[i] && w->prev[i]);
    }
    memcpy(w->prev, w->down, sizeof(w->down));
    return !w->should_close;
}

void hr_window_get_size(HrWindow* w, int* out_w, int* out_h) {
    if (out_w) *out_w = w->width;
    if (out_h) *out_h = w->height;
}

void hr_window_get_framebuffer_size(HrWindow* w, int* out_w, int* out_h) {
    hr_window_get_size(w, out_w, out_h);
}

bool hr_window_should_close(HrWindow* w) { return w->should_close; }

void hr_window_set_title(HrWindow* w, const char* title) {
    if (!title) return;
    snprintf(w->title, sizeof(w->title), "%s", title);
    wchar_t wtitle[256];
    MultiByteToWideChar(CP_UTF8, 0, title, -1, wtitle, 256);
    SetWindowTextW(w->hwnd, wtitle);
}

bool hr_key_down(HrWindow* w, HrKey key) { return key < HR_KEY_COUNT && w->down[key]; }
bool hr_key_pressed(HrWindow* w, HrKey key) { return key < HR_KEY_COUNT && w->pressed[key]; }
bool hr_key_released(HrWindow* w, HrKey key) { return key < HR_KEY_COUNT && w->released[key]; }
float hr_mouse_dx(HrWindow* w) { return w->mouse_dx; }
float hr_mouse_dy(HrWindow* w) { return w->mouse_dy; }
float hr_mouse_wheel(HrWindow* w) { return w->wheel; }

void hr_window_set_mouse_capture(HrWindow* w, bool capture) {
    if (capture == w->mouse_captured) return;
    if (capture) {
        ShowCursor(FALSE);
        RECT r;
        GetClientRect(w->hwnd, &r);
        POINT c = {(r.right - r.left) / 2, (r.bottom - r.top) / 2};
        ClientToScreen(w->hwnd, &c);
        SetCursorPos(c.x, c.y);
    } else {
        ShowCursor(TRUE);
    }
    w->mouse_captured = capture;
}

void* hr_window_native(HrWindow* w, HrNativeHandleType type) {
    switch (type) {
        case HR_NATIVE_WINDOW: return (void*)w->hwnd;
        case HR_NATIVE_HINSTANCE: return (void*)w->hinstance;
        default: return NULL;
    }
}

bool hr_file_read(const char* path, void** out_data, usize* out_size) {
    FILE* f = fopen(path, "rb");
    if (!f) return false;
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz < 0) { fclose(f); return false; }
    void* data = hr_alloc((usize)sz + 1);
    if (!data) { fclose(f); return false; }
    size_t rd = fread(data, 1, (size_t)sz, f);
    fclose(f);
    ((u8*)data)[rd] = 0;
    *out_data = data;
    if (out_size) *out_size = (usize)rd;
    return true;
}

bool hr_file_write(const char* path, const void* data, usize size) {
    FILE* f = fopen(path, "wb");
    if (!f) return false;
    size_t wr = fwrite(data, 1, size, f);
    fclose(f);
    return wr == size;
}

#endif /* _WIN32 */

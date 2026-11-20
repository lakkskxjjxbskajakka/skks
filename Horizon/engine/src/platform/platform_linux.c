#if defined(__linux__) && !defined(HR_PLATFORM_WINDOWS)

#include "horizon/platform.h"
#include "horizon/memory.h"

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/keysym.h>
#include <X11/Xatom.h>
#include <stdio.h>
#include <stdlib.h>

struct HrWindow {
    Display* display;
    Window window;
    Atom wm_delete;
    int width, height;
    int fb_width, fb_height;
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

static Display* g_display = NULL;

bool hr_platform_init(void) {
    g_display = XOpenDisplay(NULL);
    if (!g_display) {
        HR_ERROR("platform", "cannot open X11 display (DISPLAY='%s'). "
                 "On a headless machine run under Xvfb.", getenv("DISPLAY") ? getenv("DISPLAY") : "");
        return false;
    }
    HR_INFO("platform", "X11 display '%s'", DisplayString(g_display));
    return true;
}

void hr_platform_shutdown(void) {
    if (g_display) { XCloseDisplay(g_display); g_display = NULL; }
}

const char* hr_platform_name(void) { return "Linux/X11"; }

static int map_keysym(KeySym ks) {
    if (ks >= XK_a && ks <= XK_z) return HR_KEY_A + (int)(ks - XK_a);
    if (ks >= XK_0 && ks <= XK_9) return HR_KEY_0 + (int)(ks - XK_0);
    switch (ks) {
        case XK_Left: return HR_KEY_LEFT;
        case XK_Right: return HR_KEY_RIGHT;
        case XK_Up: return HR_KEY_UP;
        case XK_Down: return HR_KEY_DOWN;
        case XK_space: return HR_KEY_SPACE;
        case XK_Return: return HR_KEY_ENTER;
        case XK_Escape: return HR_KEY_ESCAPE;
        case XK_Tab: return HR_KEY_TAB;
        case XK_Shift_L: case XK_Shift_R: return HR_KEY_SHIFT;
        case XK_Control_L: case XK_Control_R: return HR_KEY_CTRL;
        case XK_Alt_L: case XK_Alt_R: return HR_KEY_ALT;
        case XK_F1: return HR_KEY_F1; case XK_F2: return HR_KEY_F2;
        case XK_F3: return HR_KEY_F3; case XK_F4: return HR_KEY_F4;
        case XK_F5: return HR_KEY_F5; case XK_F6: return HR_KEY_F6;
        case XK_F7: return HR_KEY_F7; case XK_F8: return HR_KEY_F8;
        case XK_F9: return HR_KEY_F9; case XK_F10: return HR_KEY_F10;
        case XK_F11: return HR_KEY_F11; case XK_F12: return HR_KEY_F12;
        default: return HR_KEY_UNKNOWN;
    }
}

HrWindow* hr_window_create(const HrWindowDesc* desc) {
    if (!g_display) return NULL;
    HrWindow* w = (HrWindow*)hr_calloc(1, sizeof(HrWindow));
    if (!w) return NULL;
    w->display = g_display;
    w->width = desc->width > 0 ? desc->width : 1280;
    w->height = desc->height > 0 ? desc->height : 720;
    w->hidden = desc->hidden;
    w->window = XCreateSimpleWindow(g_display, DefaultRootWindow(g_display),
                                    0, 0, (unsigned)w->width, (unsigned)w->height,
                                    0, 0, 0);
    if (!w->window) { hr_free(w); return NULL; }

    XStoreName(g_display, w->window, desc->title ? desc->title : "Horizon");
    snprintf(w->title, sizeof(w->title), "%s", desc->title ? desc->title : "Horizon");

    XSelectInput(g_display, w->window,
                 KeyPressMask | KeyReleaseMask | ButtonPressMask | ButtonReleaseMask |
                 PointerMotionMask | StructureNotifyMask | ExposureMask | FocusChangeMask);

    w->wm_delete = XInternAtom(g_display, "WM_DELETE_WINDOW", False);
    XSetWMProtocols(g_display, w->window, &w->wm_delete, 1);

    if (!desc->hidden) {
        XMapWindow(g_display, w->window);
    }
    XFlush(g_display);

    XWindowAttributes attr;
    if (XGetWindowAttributes(g_display, w->window, &attr)) {
        w->width = attr.width;
        w->height = attr.height;
    }
    w->fb_width = w->width;
    w->fb_height = w->height;
    HR_INFO("platform", "window %dx%d created (%s)", w->width, w->height,
            desc->hidden ? "hidden" : "visible");
    return w;
}

void hr_window_destroy(HrWindow* w) {
    if (!w) return;
    if (w->mouse_captured) XUngrabPointer(w->display, CurrentTime);
    XDestroyWindow(w->display, w->window);
    XFlush(w->display);
    hr_free(w);
}

static void update_deltas(HrWindow* w) {
    for (int i = 0; i < HR_KEY_COUNT; ++i) {
        w->pressed[i] = (u8)(w->down[i] && !w->prev[i]);
        w->released[i] = (u8)(!w->down[i] && w->prev[i]);
    }
    memcpy(w->prev, w->down, sizeof(w->down));
}

bool hr_window_poll(HrWindow* w) {
    w->mouse_dx = w->mouse_dy = w->wheel = 0.0f;
    while (XPending(w->display)) {
        XEvent ev;
        XNextEvent(w->display, &ev);
        switch (ev.type) {
            case KeyPress:
            case KeyRelease: {
                KeySym ks = XLookupKeysym(&ev.xkey, 0);
                int k = map_keysym(ks);
                if (k >= 0 && k < HR_KEY_COUNT) {
                    if (ev.type == KeyPress) w->down[k] = 1;
                    else w->down[k] = 0;
                }
                break;
            }
            case ButtonPress:
                if (ev.xbutton.button == Button4) w->wheel += 1.0f;
                else if (ev.xbutton.button == Button5) w->wheel -= 1.0f;
                break;
            case MotionNotify: {
                int mx = ev.xmotion.x, my = ev.xmotion.y;
                if (w->mouse_captured) {
                    int cx = w->width / 2, cy = w->height / 2;
                    w->mouse_dx += (float)(mx - cx);
                    w->mouse_dy += (float)(my - cy);
                    if (mx != cx || my != cy) {
                        XWarpPointer(w->display, None, w->window, 0, 0, 0, 0, cx, cy);
                    }
                } else if (w->have_last_mouse) {
                    w->mouse_dx += (float)(mx - w->last_mouse_x);
                    w->mouse_dy += (float)(my - w->last_mouse_y);
                }
                w->last_mouse_x = mx;
                w->last_mouse_y = my;
                w->have_last_mouse = true;
                break;
            }
            case ConfigureNotify:
                if (ev.xconfigure.width != w->width || ev.xconfigure.height != w->height) {
                    w->width = ev.xconfigure.width;
                    w->height = ev.xconfigure.height;
                    w->fb_width = w->width;
                    w->fb_height = w->height;
                }
                break;
            case ClientMessage:
                if ((Atom)ev.xclient.data.l[0] == w->wm_delete) w->should_close = true;
                break;
            default:
                break;
        }
    }
    update_deltas(w);
    return !w->should_close;
}

void hr_window_get_size(HrWindow* w, int* out_w, int* out_h) {
    if (out_w) *out_w = w->width;
    if (out_h) *out_h = w->height;
}

void hr_window_get_framebuffer_size(HrWindow* w, int* out_w, int* out_h) {
    if (out_w) *out_w = w->fb_width;
    if (out_h) *out_h = w->fb_height;
}

bool hr_window_should_close(HrWindow* w) { return w->should_close; }

void hr_window_set_title(HrWindow* w, const char* title) {
    if (!title) return;
    snprintf(w->title, sizeof(w->title), "%s", title);
    XStoreName(w->display, w->window, title);
    XFlush(w->display);
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
        XGrabPointer(w->display, w->window, True,
                     ButtonPressMask | ButtonReleaseMask | PointerMotionMask,
                     GrabModeAsync, GrabModeAsync, w->window, None, CurrentTime);
        XWarpPointer(w->display, None, w->window, 0, 0, 0, 0, w->width / 2, w->height / 2);
        XFlush(w->display);
    } else {
        XUngrabPointer(w->display, CurrentTime);
    }
    w->mouse_captured = capture;
}

void* hr_window_native(HrWindow* w, HrNativeHandleType type) {
    switch (type) {
        case HR_NATIVE_DISPLAY: return (void*)w->display;
        case HR_NATIVE_WINDOW: return (void*)(uintptr_t)w->window;
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

#endif /* __linux__ */

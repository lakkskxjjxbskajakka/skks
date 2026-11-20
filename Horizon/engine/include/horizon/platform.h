/* Horizon Engine - platform abstraction (window, input, timing, files). */
#ifndef HORIZON_PLATFORM_H
#define HORIZON_PLATFORM_H

#include "horizon/core.h"

typedef enum HrKey {
    HR_KEY_UNKNOWN = 0,
    HR_KEY_A, HR_KEY_B, HR_KEY_C, HR_KEY_D, HR_KEY_E, HR_KEY_F, HR_KEY_G,
    HR_KEY_H, HR_KEY_I, HR_KEY_J, HR_KEY_K, HR_KEY_L, HR_KEY_M, HR_KEY_N,
    HR_KEY_O, HR_KEY_P, HR_KEY_Q, HR_KEY_R, HR_KEY_S, HR_KEY_T, HR_KEY_U,
    HR_KEY_V, HR_KEY_W, HR_KEY_X, HR_KEY_Y, HR_KEY_Z,
    HR_KEY_0, HR_KEY_1, HR_KEY_2, HR_KEY_3, HR_KEY_4,
    HR_KEY_5, HR_KEY_6, HR_KEY_7, HR_KEY_8, HR_KEY_9,
    HR_KEY_LEFT, HR_KEY_RIGHT, HR_KEY_UP, HR_KEY_DOWN,
    HR_KEY_SPACE, HR_KEY_ENTER, HR_KEY_ESCAPE, HR_KEY_TAB,
    HR_KEY_SHIFT, HR_KEY_CTRL, HR_KEY_ALT,
    HR_KEY_F1, HR_KEY_F2, HR_KEY_F3, HR_KEY_F4, HR_KEY_F5, HR_KEY_F6,
    HR_KEY_F7, HR_KEY_F8, HR_KEY_F9, HR_KEY_F10, HR_KEY_F11, HR_KEY_F12,
    HR_KEY_COUNT
} HrKey;

typedef struct HrWindow HrWindow;

typedef struct HrWindowDesc {
    const char* title;
    int width;
    int height;
    bool hidden;
    bool resizable;
} HrWindowDesc;

bool hr_platform_init(void);
void hr_platform_shutdown(void);
const char* hr_platform_name(void);

HrWindow* hr_window_create(const HrWindowDesc* desc);
void hr_window_destroy(HrWindow* w);

/* Returns false when the window should close. */
bool hr_window_poll(HrWindow* w);
void hr_window_get_size(HrWindow* w, int* out_w, int* out_h);
void hr_window_get_framebuffer_size(HrWindow* w, int* out_w, int* out_h);
bool hr_window_should_close(HrWindow* w);
void hr_window_set_title(HrWindow* w, const char* title);

bool hr_key_down(HrWindow* w, HrKey key);
bool hr_key_pressed(HrWindow* w, HrKey key);
bool hr_key_released(HrWindow* w, HrKey key);
float hr_mouse_dx(HrWindow* w);
float hr_mouse_dy(HrWindow* w);
float hr_mouse_wheel(HrWindow* w);
void hr_window_set_mouse_capture(HrWindow* w, bool capture);

/* Native handles for the graphics backend. */
typedef enum HrNativeHandleType {
    HR_NATIVE_DISPLAY = 0,
    HR_NATIVE_WINDOW = 1,
    HR_NATIVE_HINSTANCE = 2
} HrNativeHandleType;
void* hr_window_native(HrWindow* w, HrNativeHandleType type);

bool hr_file_read(const char* path, void** out_data, usize* out_size);
bool hr_file_write(const char* path, const void* data, usize size);

#endif /* HORIZON_PLATFORM_H */

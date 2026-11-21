/* Horizon Engine - playable sunset driving demo / benchmark harness. */
#include "horizon/platform.h"
#include "horizon/renderer.h"
#include "horizon/jobs.h"
#include "horizon/memory.h"
#include "horizon/core.h"
#include "world.h"

#include <errno.h>
#include <sys/stat.h>

typedef struct DemoOptions {
    int width, height;
    int frames;
    int shot_frame;
    int shot_every;
    int shadow_res;
    float scale;
    bool autopilot;
    bool vsync;
    bool hidden;
    bool fixed_dt;
    bool adaptive;
    bool validation;
    const char* screenshot;
    const char* shots_dir;
    const char* benchmark;
} DemoOptions;

static void ensure_dir(const char* path) {
#if defined(_WIN32)
    _mkdir(path);
#else
    if (mkdir(path, 0755) != 0 && errno != EEXIST) {
        HR_WARN("demo", "could not create directory %s", path);
    }
#endif
}

static void parse_args(DemoOptions* o, int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        if (!strcmp(a, "--width") && i + 1 < argc) o->width = atoi(argv[++i]);
        else if (!strcmp(a, "--height") && i + 1 < argc) o->height = atoi(argv[++i]);
        else if (!strcmp(a, "--frames") && i + 1 < argc) o->frames = atoi(argv[++i]);
        else if (!strcmp(a, "--shot-frame") && i + 1 < argc) o->shot_frame = atoi(argv[++i]);
        else if (!strcmp(a, "--shot-every") && i + 1 < argc) o->shot_every = atoi(argv[++i]);
        else if (!strcmp(a, "--shadow-res") && i + 1 < argc) o->shadow_res = atoi(argv[++i]);
        else if (!strcmp(a, "--scale") && i + 1 < argc) o->scale = (float)atof(argv[++i]);
        else if (!strcmp(a, "--screenshot") && i + 1 < argc) o->screenshot = argv[++i];
        else if (!strcmp(a, "--shots") && i + 1 < argc) o->shots_dir = argv[++i];
        else if (!strcmp(a, "--benchmark") && i + 1 < argc) o->benchmark = argv[++i];
        else if (!strcmp(a, "--auto")) o->autopilot = true;
        else if (!strcmp(a, "--vsync")) o->vsync = true;
        else if (!strcmp(a, "--no-vsync")) o->vsync = false;
        else if (!strcmp(a, "--hidden")) o->hidden = true;
        else if (!strcmp(a, "--fixed-dt")) o->fixed_dt = true;
        else if (!strcmp(a, "--fixed-quality")) o->adaptive = false;
        else if (!strcmp(a, "--validate")) o->validation = true;
        else if (!strcmp(a, "--help")) {
            printf("usage: horizon_demo [--width N] [--height N] [--frames N] [--auto]\n"
                   "  [--screenshot FILE] [--shot-frame N] [--shots DIR --shot-every N]\n"
                   "  [--benchmark FILE] [--scale S] [--shadow-res N] [--vsync|--no-vsync]\n"
                   "  [--fixed-dt] [--fixed-quality] [--hidden] [--validate]\n");
            exit(0);
        }
    }
    if (o->frames > 0 && o->shot_frame < 0) o->shot_frame = o->frames - 1;
    if (o->screenshot && o->frames == 0) o->frames = 240;
    if (o->screenshot && o->shot_frame < 0) o->shot_frame = 180;
}

int main(int argc, char** argv) {
    DemoOptions opt;
    memset(&opt, 0, sizeof(opt));
    opt.width = 1600; opt.height = 900;
    opt.frames = 0;
    opt.shot_frame = -1;
    opt.shot_every = 0;
    opt.shadow_res = 2048;
    opt.scale = 1.0f;
    opt.autopilot = false;
    opt.vsync = true;
    opt.hidden = false;
    opt.fixed_dt = false;
    opt.adaptive = true;
    opt.validation = false;
    parse_args(&opt, argc, argv);

    if (opt.frames > 0) { opt.autopilot = true; opt.vsync = false; opt.fixed_dt = true; }
    if (opt.screenshot || opt.shots_dir || opt.benchmark) opt.vsync = false;
    if (opt.shots_dir) ensure_dir(opt.shots_dir);
    ensure_dir("screenshots");

    hr_log_set_level(HR_LOG_INFO_LEVEL);
    if (!hr_platform_init()) {
        fprintf(stderr, "failed to initialize platform\n");
        return 1;
    }

    HrWindowDesc wd;
    memset(&wd, 0, sizeof(wd));
    wd.title = "Horizon - Sunset Drive";
    wd.width = opt.width;
    wd.height = opt.height;
    wd.hidden = opt.hidden;
    wd.resizable = true;
    HrWindow* window = hr_window_create(&wd);
    if (!window) {
        fprintf(stderr, "failed to create window\n");
        hr_platform_shutdown();
        return 1;
    }

    HrRendererDesc rd;
    memset(&rd, 0, sizeof(rd));
    rd.window = window;
    rd.vsync = opt.vsync;
    rd.validation = opt.validation;
    rd.shadow_resolution = opt.shadow_res;
    HrRenderer* renderer = NULL;
    if (!hr_renderer_init(&renderer, &rd)) {
        fprintf(stderr, "failed to initialize renderer\n");
        hr_window_destroy(window);
        hr_platform_shutdown();
        return 1;
    }

    HrJobSystem* jobs = NULL;
    hr_jobs_init(&jobs, 0);
    HrGameWorld* world = NULL;
    if (!hr_game_world_create(&world, renderer, jobs)) {
        fprintf(stderr, "failed to build world\n");
        hr_jobs_shutdown(jobs);
        hr_renderer_shutdown(renderer);
        hr_window_destroy(window);
        hr_platform_shutdown();
        return 1;
    }

    FILE* bench = NULL;
    if (opt.benchmark) {
        bench = fopen(opt.benchmark, "w");
        if (bench) {
            fprintf(bench, "frame,cpu_ms,gpu_ms,fps,draws,instances,triangles,hdr_w,hdr_h,scale,shadow_res\n");
        } else {
            HR_WARN("demo", "could not open benchmark file %s", opt.benchmark);
        }
    }

    float render_scale = HR_CLAMP(opt.scale, 0.45f, 1.0f);
    int shadow_res = HR_CLAMP(opt.shadow_res, 512, 4096);
    float ema_ms = 16.0f;
    int quality_cooldown = 0;
    const float fixed_dt = 1.0f / 60.0f;
    double last_time = hr_time_sec();
    double sim_time = 0.0;
    u64 frame = 0;
    int exit_code = 0;
    double sum_ms = 0.0, min_ms = 1e9, max_ms = 0.0;
    int bench_samples = 0;
    int title_tick = 0;
    bool manual_shot = false;
    char shot_path[512];
    char title[256];

    HR_INFO("demo", "controls: W/S drive, A/D steer, Space handbrake, Shift boost, R reset, F1 autopilot, F12 screenshot, Esc quit");

    while (true) {
        if (!hr_window_poll(window) || hr_window_should_close(window)) break;
        if (hr_key_pressed(window, HR_KEY_ESCAPE)) break;

        if (hr_key_pressed(window, HR_KEY_F1)) opt.autopilot = !opt.autopilot;
        if (hr_key_pressed(window, HR_KEY_R)) hr_game_world_reset(world);
        if (hr_key_pressed(window, HR_KEY_F12)) {
            snprintf(shot_path, sizeof(shot_path), "screenshots/horizon_%04llu.bmp",
                     (unsigned long long)frame);
            manual_shot = true;
        }

        double now = hr_time_sec();
        float dt = opt.fixed_dt ? fixed_dt : (float)HR_CLAMP(now - last_time, 0.0, 0.05);
        last_time = now;
        if (dt <= 0.0f) dt = fixed_dt;

        HrGameWorldInput in;
        memset(&in, 0, sizeof(in));
        in.autopilot = opt.autopilot;
        if (!opt.autopilot) {
            float th = (hr_key_down(window, HR_KEY_W) || hr_key_down(window, HR_KEY_UP)) ? 1.0f : 0.0f;
            float br = (hr_key_down(window, HR_KEY_S) || hr_key_down(window, HR_KEY_DOWN)) ? 1.0f : 0.0f;
            float st = 0.0f;
            if (hr_key_down(window, HR_KEY_D) || hr_key_down(window, HR_KEY_RIGHT)) st += 1.0f;
            if (hr_key_down(window, HR_KEY_A) || hr_key_down(window, HR_KEY_LEFT)) st -= 1.0f;
            in.throttle = th;
            in.brake = br;
            in.steer = st;
            in.handbrake = hr_key_down(window, HR_KEY_SPACE);
            in.boost = hr_key_down(window, HR_KEY_SHIFT);
        }
        hr_game_world_update(world, dt, &in);
        sim_time += dt;

        HrFrameDesc fd;
        hr_game_world_describe_frame(world, &fd, render_scale, (float)sim_time);
        fd.shadow_resolution = shadow_res;
        fd.screenshot_path = NULL;
        if (manual_shot) {
            fd.screenshot_path = shot_path;
            manual_shot = false;
        }
        if (opt.screenshot && (int)frame == opt.shot_frame) fd.screenshot_path = opt.screenshot;
        if (opt.shots_dir && opt.shot_every > 0 && frame >= 60 && (frame - 60) % (u64)opt.shot_every == 0) {
            snprintf(shot_path, sizeof(shot_path), "%s/frame_%04llu.bmp", opt.shots_dir,
                     (unsigned long long)frame);
            fd.screenshot_path = shot_path;
        }

        if (hr_renderer_begin_frame(renderer, &fd)) {
            hr_game_world_render(world);
            hr_renderer_end_frame(renderer);
        }

        HrRendererStats st;
        hr_renderer_get_stats(renderer, &st);
        double ms = st.cpu_frame_ms;
        if (st.gpu_frame_ms > ms) ms = st.gpu_frame_ms;
        if (ms <= 0.0) ms = 16.6;
        ema_ms = ema_ms * 0.90f + (float)ms * 0.10f;

        if (frame > 30) {
            sum_ms += ms;
            if (ms < min_ms) min_ms = ms;
            if (ms > max_ms) max_ms = ms;
            bench_samples++;
        }

        if (bench) {
            fprintf(bench, "%llu,%.3f,%.3f,%.1f,%u,%u,%u,%u,%u,%.2f,%d\n",
                    (unsigned long long)frame, st.cpu_frame_ms, st.gpu_frame_ms,
                    st.cpu_frame_ms > 0.0 ? 1000.0 / st.cpu_frame_ms : 0.0,
                    st.draw_calls, st.instances, st.triangles,
                    st.hdr_width, st.hdr_height, render_scale, shadow_res);
            if ((frame % 60) == 0) fflush(bench);
        }

        if (opt.adaptive && frame > 40 && !opt.screenshot) {
            if (quality_cooldown > 0) quality_cooldown--;
            else if (ema_ms > 19.0f) {
                if (render_scale > 0.62f) { render_scale -= 0.06f; quality_cooldown = 24; }
                else if (shadow_res > 1024) { shadow_res /= 2; quality_cooldown = 40; }
            } else if (ema_ms < 12.5f) {
                if (shadow_res < 2048) { shadow_res *= 2; quality_cooldown = 90; }
                else if (render_scale < 1.0f) { render_scale += 0.04f; quality_cooldown = 90; }
            }
        }

        title_tick++;
        if (title_tick >= 15) {
            title_tick = 0;
            double fps = st.cpu_frame_ms > 0.0 ? 1000.0 / st.cpu_frame_ms : 0.0;
            snprintf(title, sizeof(title),
                     "Horizon | %s | %.0f km/h | %s | %.0f fps | %u draws | scale %.2f",
                     st.backend ? st.backend : "vk", hr_game_world_speed_kph(world),
                     hr_game_world_area(world), fps, st.draw_calls, render_scale);
            hr_window_set_title(window, title);
        }

        if (opt.frames > 0 && (int)frame >= opt.frames - 1) break;
        frame++;

        if (opt.frames == 0 && (frame % 600) == 0) {
            HR_INFO("demo", "%.0f fps avg, %.0f km/h, %u draws, %u instances",
                    1000.0 / HR_MAX(ema_ms, 0.001f), hr_game_world_speed_kph(world),
                    st.draw_calls, st.instances);
        }
    }

    if (bench_samples > 0) {
        HR_INFO("demo", "benchmark: %d frames, avg %.2f ms (%.1f fps), min %.2f ms, max %.2f ms",
                bench_samples, sum_ms / bench_samples, 1000.0 / (sum_ms / bench_samples),
                min_ms, max_ms);
    }
    if (bench) fclose(bench);

    hr_game_world_destroy(world);
    hr_jobs_shutdown(jobs);
    hr_renderer_shutdown(renderer);
    hr_window_destroy(window);
    hr_platform_shutdown();
    return exit_code;
}

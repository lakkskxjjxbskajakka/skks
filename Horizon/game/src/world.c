/* Horizon Engine - driving world state, arcade vehicle dynamics and camera. */
#include "world_internal.h"
#include "horizon/memory.h"
#include <math.h>

static float wrap_angle(float a) {
    while (a > HR_PI) a -= HR_TAU;
    while (a < -HR_PI) a += HR_TAU;
    return a;
}

bool hr_world_create(HrWorld** out, HrRenderer* r, HrJobSystem* js) {
    HrWorld* w = (HrWorld*)hr_calloc(1, sizeof(HrWorld));
    if (!w) return false;
    w->renderer = r;
    w->jobs = js;

    w->sun_direction = v3_norm(v3(-0.75f, -0.17f, -0.64f));
    w->sun_color = v3(1.0f, 0.70f, 0.44f);
    w->ambient_sky = v3(0.16f, 0.26f, 0.47f);
    w->ambient_ground = v3(0.11f, 0.085f, 0.065f);
    w->fog_color = v3(0.70f, 0.46f, 0.31f);
    w->fog_density = 0.00155f;
    w->cam_fov = 58.0f;

    hr_world_build_environment(w);
    hr_world_build_car(w);
    hr_world_reset(w);
    *out = w;
    return true;
}

void hr_world_destroy(HrWorld* w) {
    if (!w) return;
    hr_free(w->road);
    hr_free(w->trees);
    hr_free(w->rocks);
    hr_free(w->posts);
    hr_free(w);
}

void hr_world_reset(HrWorld* w) {
    if (!w || w->road_count == 0) return;
    const HrRoadSample* s = &w->road[0];
    w->car_pos = s->pos;
    w->car_yaw = atan2f(s->tangent.x, s->tangent.z);
    w->car_speed = 0.0f;
    w->car_steer = 0.0f;
    w->car_wheel_spin = 0.0f;
    w->car_pitch = 0.0f;
    w->car_roll = 0.0f;
    w->car_bob = 0.0f;
    w->car_y = hr_ground_height(w, w->car_pos.x, w->car_pos.z) + 0.02f;
    w->car_forward = v3(sinf(w->car_yaw), 0.0f, cosf(w->car_yaw));
    w->car_right = v3(cosf(w->car_yaw), 0.0f, -sinf(w->car_yaw));
    w->distance_traveled = 0.0f;
    w->progress = 0.0f;
    w->offroad = 0.0f;
    w->nearest = 0;
    w->brake_lights = false;
    w->cam_pos = v3_sub(v3_add(w->car_pos, v3(0, 3.2f, 0)), v3_scale(w->car_forward, 7.2f));
    w->cam_target = v3_add(w->car_pos, v3(0, 1.0f, 0));
    w->cam_shake = 0.0f;
}

static void autopilot(HrWorld* w, float* throttle, float* steer, float* brake) {
    float look_m = 12.0f + fabsf(w->car_speed) * 0.52f;
    float t = w->progress + look_m / HR_MAX(w->road_length, 1.0f);
    Vec3 target = hr_road_point(w, t);
    float dx = target.x - w->car_pos.x;
    float dz = target.z - w->car_pos.z;
    float desired = atan2f(dx, dz);
    float diff = wrap_angle(desired - w->car_yaw);
    *steer = HR_CLAMP(diff * 2.4f, -1.0f, 1.0f);

    float curv = 0.0f;
    for (int i = 4; i <= 48; i += 4) {
        int idx = (w->nearest + i) % w->road_count;
        curv = HR_MAX(curv, w->road[idx].curvature);
    }
    float target_speed = 62.0f;
    if (curv > 1e-4f) target_speed = sqrtf(14.0f / curv);
    target_speed = HR_CLAMP(target_speed, 20.0f, 60.0f);

    float s = fabsf(w->car_speed);
    if (s < target_speed - 1.0f) { *throttle = 1.0f; *brake = 0.0f; }
    else if (s > target_speed + 5.0f) { *throttle = 0.0f; *brake = 1.0f; }
    else { *throttle = 0.35f; *brake = 0.0f; }
}

void hr_world_update(HrWorld* w, float dt, const HrWorldInput* input) {
    if (!w || dt <= 0.0f) return;
    dt = HR_MIN(dt, 0.05f);
    w->time += dt;

    float throttle = input->throttle;
    float steer_in = input->steer;
    float brake = input->brake;
    if (input->autopilot) autopilot(w, &throttle, &steer_in, &brake);

    float speed = w->car_speed;
    float max_speed = 64.0f * (input->boost ? 1.16f : 1.0f) * (1.0f - w->offroad * 0.5f);
    if (throttle > 0.01f) {
        speed += throttle * 12.5f * HR_MAX(1.0f - speed / max_speed, 0.0f) * dt;
    } else if (throttle < -0.01f) {
        speed += throttle * 17.0f * dt;
        speed = HR_MAX(speed, -10.0f);
    } else {
        speed -= hr_sign(speed) * HR_MIN(fabsf(speed), 1.1f * dt);
    }
    if (brake > 0.01f) speed -= hr_sign(speed) * HR_MIN(fabsf(speed), brake * 24.0f * dt);
    float drag = 0.35f + 0.00115f * speed * speed + w->offroad * 5.0f;
    speed -= hr_sign(speed) * HR_MIN(fabsf(speed), drag * dt);
    if (input->handbrake) speed -= hr_sign(speed) * HR_MIN(fabsf(speed), 10.0f * dt);
    speed = HR_CLAMP(speed, -10.0f, 78.0f);
    w->car_speed = speed;

    float speed_abs = fabsf(speed);
    float max_steer = 0.60f / (1.0f + speed_abs * 0.055f);
    float steer_angle = steer_in * max_steer;
    w->car_steer = hr_damp(w->car_steer, steer_in, 9.0f, dt);
    float grip = input->handbrake ? 0.55f : (1.0f - w->offroad * 0.25f);
    float yaw_rate = (speed / 2.72f) * tanf(steer_angle) * 1.08f * grip;
    float lim = (input->handbrake ? 28.0f : 17.5f) / HR_MAX(speed_abs, 4.0f);
    yaw_rate = HR_CLAMP(yaw_rate, -lim, lim);
    w->car_yaw += yaw_rate * dt;
    w->car_yaw = wrap_angle(w->car_yaw);

    w->car_forward = v3(sinf(w->car_yaw), 0.0f, cosf(w->car_yaw));
    w->car_right = v3(cosf(w->car_yaw), 0.0f, -sinf(w->car_yaw));
    float moved = speed * dt;
    w->car_pos = v3_add(w->car_pos, v3_scale(w->car_forward, moved));
    if (fabsf(moved) > 1e-4f) w->distance_traveled += fabsf(moved);

    float dist = 0.0f;
    int nearest = hr_road_nearest(w, w->car_pos.x, w->car_pos.z, &dist);
    w->nearest = nearest;
    w->progress = (float)nearest / (float)w->road_count;
    float off = hr_smoothstep(HR_ROAD_HALF_WIDTH + 0.4f, HR_ROAD_HALF_WIDTH + 4.5f, dist);
    w->offroad = hr_damp(w->offroad, off, 4.0f, dt);

    float gh = hr_ground_height(w, w->car_pos.x, w->car_pos.z);
    w->car_y = hr_damp(w->car_y, gh + 0.02f, 16.0f, dt);

    Vec3 fwd = w->car_forward;
    Vec3 rgt = w->car_right;
    float hf = hr_ground_height(w, w->car_pos.x + fwd.x * 1.35f, w->car_pos.z + fwd.z * 1.35f);
    float hb = hr_ground_height(w, w->car_pos.x - fwd.x * 1.30f, w->car_pos.z - fwd.z * 1.30f);
    float hr = hr_ground_height(w, w->car_pos.x + rgt.x * 0.9f, w->car_pos.z + rgt.z * 0.9f);
    float hl = hr_ground_height(w, w->car_pos.x - rgt.x * 0.9f, w->car_pos.z - rgt.z * 0.9f);
    float pitch_target = atan2f(hb - hf, 2.65f);
    float roll_target = atan2f(hr - hl, 1.8f) + steer_in * 0.035f * HR_CLAMP(speed_abs / 30.0f, 0.0f, 1.0f);
    w->car_pitch = hr_damp(w->car_pitch, pitch_target, 9.0f, dt);
    w->car_roll = hr_damp(w->car_roll, roll_target, 9.0f, dt);
    w->car_wheel_spin += (speed / 0.365f) * dt;
    w->car_bob = sinf(w->time * 17.0f) * 0.0035f * HR_CLAMP(speed_abs / 40.0f, 0.0f, 1.0f);

    w->brake_lights = (brake > 0.05f) || (throttle < -0.05f) || (input->handbrake && speed > 0.5f);
    w->boost_active = input->boost;

    Vec3 cam_desired = v3_sub(w->car_pos, v3_scale(w->car_forward, 7.1f));
    cam_desired = v3_add(cam_desired, v3(0.0f, 3.15f, 0.0f));
    cam_desired = v3_add(cam_desired, v3_scale(w->car_right, -steer_in * 0.55f));
    float ground_cam = hr_ground_height(w, cam_desired.x, cam_desired.z) + 1.1f;
    cam_desired.y = HR_MAX(cam_desired.y, ground_cam);
    w->cam_shake = hr_damp(w->cam_shake, w->offroad * 0.035f + (input->boost ? 0.01f : 0.0f), 5.0f, dt);
    float sh = w->cam_shake;
    cam_desired = v3_add(cam_desired, v3(sinf(w->time * 47.0f) * sh,
                                          sinf(w->time * 61.0f) * sh * 1.4f,
                                          cosf(w->time * 53.0f) * sh));
    w->cam_pos = v3_damp(w->cam_pos, cam_desired, 6.5f, dt);

    Vec3 target = v3_add(w->car_pos, v3_scale(w->car_forward, 5.6f + speed_abs * 0.085f));
    target.y = w->car_y + 1.05f;
    w->cam_target = v3_damp(w->cam_target, target, 9.0f, dt);
    float fov_target = 57.0f + HR_CLAMP(speed_abs, 0.0f, 70.0f) * 0.22f;
    w->cam_fov = hr_damp(w->cam_fov, fov_target, 3.0f, dt);

    if (w->car_pos.y > 500.0f || w->car_pos.y < -500.0f) hr_world_reset(w);
}

void hr_world_render(HrWorld* w) {
    HrRenderer* r = w->renderer;
    hr_renderer_draw(r, w->mesh_terrain, HR_MAT_TERRAIN, mat4_identity(), v4s(1));
    hr_renderer_draw(r, w->mesh_road, HR_MAT_ROAD, mat4_identity(), v4(0.60f, 0.60f, 0.64f, 1));
    hr_renderer_draw(r, w->mesh_shoulder, HR_MAT_CONCRETE, mat4_identity(),
                     v4(0.44f, 0.40f, 0.34f, 1));

    for (int i = 0; i < w->tree_count; ++i) {
        const HrTreeInstance* t = &w->trees[i];
        Mat4 model = mat4_mul(mat4_translate(t->pos),
                              mat4_mul(mat4_rotate_y(t->rotation), mat4_scale_uniform(t->scale)));
        hr_renderer_draw(r, w->mesh_trunk, HR_MAT_TRUNK, model, v4(1, 1, 1, 1));
        hr_renderer_draw(r, t->kind ? w->mesh_leaf : w->mesh_pine, HR_MAT_FOLIAGE, model, t->tint);
    }
    for (int i = 0; i < w->rock_count; ++i) {
        const HrRockInstance* rock = &w->rocks[i];
        Mat4 model = mat4_mul(mat4_translate(rock->pos),
                              mat4_mul(mat4_rotate_y(rock->rotation), mat4_scale(rock->scale)));
        hr_renderer_draw(r, w->mesh_rock, HR_MAT_ROCK, model, rock->tint);
    }
    for (int i = 0; i < w->post_count; ++i) {
        Mat4 model = mat4_mul(mat4_translate(w->posts[i].pos), mat4_rotate_y(w->posts[i].rotation));
        hr_renderer_draw(r, w->mesh_post, HR_MAT_CONCRETE, model, v4(0.42f, 0.41f, 0.39f, 1));
    }
    hr_world_draw_car(w, r);
}

void hr_world_describe_frame(HrWorld* w, HrFrameDesc* f, float render_scale, float time) {
    memset(f, 0, sizeof(*f));
    f->camera_position = w->cam_pos;
    f->camera_target = w->cam_target;
    f->camera_up = v3(0, 1, 0);
    f->fov_deg = w->cam_fov;
    f->near_plane = 0.08f;
    f->far_plane = 1500.0f;
    f->focus = w->car_pos;
    f->sun_direction = w->sun_direction;
    f->sun_color = w->sun_color;
    f->sun_intensity = 0.0f;
    f->ambient_sky = w->ambient_sky;
    f->ambient_ground = w->ambient_ground;
    f->exposure = 1.2f;
    f->time = time;
    f->fog_color = w->fog_color;
    f->fog_density = w->fog_density;
    f->render_scale = render_scale;
    f->shadow_distance = 85.0f;
    f->shadows_enabled = 0;
    f->bloom_enabled = 0;
    f->fxaa_enabled = 1;
    f->shadow_resolution = 2048;
    f->bloom_intensity = 0.8f;
    f->saturation = 1.10f;
    f->contrast = 1.04f;
    f->vignette = 0.34f;
    f->grain = 0.010f;
}

float hr_world_speed_kph(const HrWorld* w) { return fabsf(w->car_speed) * 3.6f; }
float hr_world_distance(const HrWorld* w) { return w->distance_traveled; }
Vec3 hr_world_car_position(const HrWorld* w) { return w->car_pos; }
float hr_world_progress(const HrWorld* w) { return w->progress; }

const char* hr_world_area(const HrWorld* w) {
    float p = w->progress;
    if (p < 0.16f) return "Cypress Straight";
    if (p < 0.32f) return "Sunset Ridge";
    if (p < 0.48f) return "The Carousel";
    if (p < 0.64f) return "Valley Drop";
    if (p < 0.80f) return "Pine Hairpin";
    if (p < 0.94f) return "Golden Mile";
    return "Start Finish";
}

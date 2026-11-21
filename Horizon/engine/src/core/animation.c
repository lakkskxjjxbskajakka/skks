/* Horizon Engine - skeletal animation sampling, blending and cross-fade. */
#include "horizon/animation.h"
#include "horizon/memory.h"

#include <math.h>

void hr_skeleton_init(HrSkeleton* skel) {
    if (!skel) return;
    memset(skel, 0, sizeof(*skel));
}

i32 hr_skeleton_add_joint(HrSkeleton* skel, const char* name, i32 parent, Mat4 inverse_bind) {
    if (!skel || skel->joint_count >= HR_ANIM_MAX_JOINTS) return -1;
    if (parent >= (i32)skel->joint_count) return -1;
    u32 idx = skel->joint_count++;
    HrJoint* j = &skel->joints[idx];
    memset(j, 0, sizeof(*j));
    if (name) snprintf(j->name, sizeof(j->name), "%s", name);
    j->parent = parent;
    j->inverse_bind = inverse_bind;
    return (i32)idx;
}

i32 hr_skeleton_find_joint(const HrSkeleton* skel, const char* name) {
    if (!skel || !name) return -1;
    for (u32 i = 0; i < skel->joint_count; ++i) {
        if (strcmp(skel->joints[i].name, name) == 0) return (i32)i;
    }
    return -1;
}

void hr_pose_identity(HrPose* pose, u32 joint_count) {
    if (!pose) return;
    memset(pose, 0, sizeof(*pose));
    pose->joint_count = joint_count < HR_ANIM_MAX_JOINTS ? joint_count : HR_ANIM_MAX_JOINTS;
    for (u32 i = 0; i < pose->joint_count; ++i) {
        pose->translations[i] = v3(0, 0, 0);
        pose->rotations[i] = quat_identity();
        pose->scales[i] = v3s(1.0f);
    }
}

void hr_pose_bind(const HrSkeleton* skel, HrPose* out_pose) {
    if (!skel || !out_pose) return;
    hr_pose_identity(out_pose, skel->joint_count);
}

static Quat quat_nlerp(Quat a, Quat b, float t) {
    if (a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w < 0.0f) {
        b.x = -b.x; b.y = -b.y; b.z = -b.z; b.w = -b.w;
    }
    Quat q = {{hr_lerp(a.x, b.x, t), hr_lerp(a.y, b.y, t),
               hr_lerp(a.z, b.z, t), hr_lerp(a.w, b.w, t)}};
    return quat_norm(q);
}

void hr_pose_lerp(const HrPose* a, const HrPose* b, float t, HrPose* out) {
    if (!a || !b || !out) return;
    u32 n = HR_MAX(a->joint_count, b->joint_count);
    if (n > HR_ANIM_MAX_JOINTS) n = HR_ANIM_MAX_JOINTS;
    out->joint_count = n;
    for (u32 i = 0; i < n; ++i) {
        Vec3 ta = i < a->joint_count ? a->translations[i] : v3(0, 0, 0);
        Vec3 tb = i < b->joint_count ? b->translations[i] : v3(0, 0, 0);
        Quat ra = i < a->joint_count ? a->rotations[i] : quat_identity();
        Quat rb = i < b->joint_count ? b->rotations[i] : quat_identity();
        Vec3 sa = i < a->joint_count ? a->scales[i] : v3s(1.0f);
        Vec3 sb = i < b->joint_count ? b->scales[i] : v3s(1.0f);
        out->translations[i] = v3_lerp(ta, tb, t);
        out->rotations[i] = quat_nlerp(ra, rb, t);
        out->scales[i] = v3_lerp(sa, sb, t);
    }
}

void hr_pose_local_matrices(const HrPose* pose, Mat4* out_local, u32 count) {
    if (!pose || !out_local) return;
    u32 n = HR_MIN(count, pose->joint_count);
    for (u32 i = 0; i < n; ++i) {
        out_local[i] = mat4_compose(pose->translations[i], pose->rotations[i], pose->scales[i]);
    }
}

void hr_pose_palette(const HrSkeleton* skel, const HrPose* pose, Mat4* out_palette) {
    if (!skel || !pose || !out_palette) return;
    u32 n = HR_MIN(skel->joint_count, pose->joint_count);
    Mat4 globals[HR_ANIM_MAX_JOINTS];
    for (u32 i = 0; i < n; ++i) {
        Mat4 local = mat4_compose(pose->translations[i], pose->rotations[i], pose->scales[i]);
        i32 p = skel->joints[i].parent;
        if (p >= 0 && (u32)p < i) globals[i] = mat4_mul(globals[p], local);
        else globals[i] = local;
        out_palette[i] = mat4_mul(globals[i], skel->joints[i].inverse_bind);
    }
    for (u32 i = n; i < skel->joint_count; ++i) out_palette[i] = mat4_identity();
}

HrAnimationClip* hr_animation_clip_create(const char* name, float duration, bool looping) {
    HrAnimationClip* clip = (HrAnimationClip*)hr_calloc(1, sizeof(HrAnimationClip));
    if (!clip) return NULL;
    if (name) snprintf(clip->name, sizeof(clip->name), "%s", name);
    clip->duration = duration > 0.0f ? duration : 0.0f;
    clip->looping = looping;
    return clip;
}

void hr_animation_clip_destroy(HrAnimationClip* clip) {
    if (!clip) return;
    for (u32 i = 0; i < clip->track_count; ++i) {
        HrAnimTrack* t = &clip->tracks[i];
        hr_free(t->positions);
        hr_free(t->rotations);
        hr_free(t->scales);
    }
    hr_free(clip->tracks);
    hr_free(clip);
}

HrAnimTrack* hr_animation_clip_add_track(HrAnimationClip* clip, u32 joint_index) {
    if (!clip) return NULL;
    if (clip->track_count >= clip->track_capacity) {
        u32 cap = clip->track_capacity ? clip->track_capacity * 2u : 8u;
        HrAnimTrack* tracks = (HrAnimTrack*)hr_realloc(clip->tracks, cap * sizeof(HrAnimTrack));
        if (!tracks) return NULL;
        clip->tracks = tracks;
        clip->track_capacity = cap;
    }
    HrAnimTrack* t = &clip->tracks[clip->track_count++];
    memset(t, 0, sizeof(*t));
    t->joint_index = joint_index;
    return t;
}

bool hr_anim_track_add_position(HrAnimTrack* track, float time, Vec3 value) {
    if (!track) return false;
    u32 cap = track->position_count; /* grow policy: track capacities not stored, use count as cap heuristic? */
    /* keep separate capacities by over-alloc strategy: store cap in count growth. Simpler: always realloc exact+growth */
    u32 ncap = track->position_count ? track->position_count + 1 : 1;
    /* We don't store cap; use a simple approach: realloc to count+1 each time is bad but simple. Better keep caps. */
    (void)cap;
    /* implement with temporary: use push helper with local cap tracking not possible. Let's just realloc. */
    HrAnimKeyVec3* n = (HrAnimKeyVec3*)hr_realloc(track->positions, (usize)(track->position_count + 1) * sizeof(HrAnimKeyVec3));
    if (!n) return false;
    track->positions = n;
    track->positions[track->position_count].time = time;
    track->positions[track->position_count].value = value;
    track->position_count++;
    return true;
}

bool hr_anim_track_add_rotation(HrAnimTrack* track, float time, Quat value) {
    if (!track) return false;
    HrAnimKeyQuat* n = (HrAnimKeyQuat*)hr_realloc(track->rotations, (usize)(track->rotation_count + 1) * sizeof(HrAnimKeyQuat));
    if (!n) return false;
    track->rotations = n;
    track->rotations[track->rotation_count].time = time;
    track->rotations[track->rotation_count].value = quat_norm(value);
    track->rotation_count++;
    return true;
}

bool hr_anim_track_add_scale(HrAnimTrack* track, float time, Vec3 value) {
    if (!track) return false;
    HrAnimKeyVec3* n = (HrAnimKeyVec3*)hr_realloc(track->scales, (usize)(track->scale_count + 1) * sizeof(HrAnimKeyVec3));
    if (!n) return false;
    track->scales = n;
    track->scales[track->scale_count].time = time;
    track->scales[track->scale_count].value = value;
    track->scale_count++;
    return true;
}

static float wrap_time(float t, float duration, bool looping) {
    if (duration <= 0.0f) return 0.0f;
    if (looping) {
        t = fmodf(t, duration);
        if (t < 0.0f) t += duration;
        return t;
    }
    return HR_CLAMP(t, 0.0f, duration);
}

static Vec3 sample_vec3(const HrAnimKeyVec3* keys, u32 count, float time, Vec3 fallback) {
    if (count == 0 || !keys) return fallback;
    if (time <= keys[0].time) return keys[0].value;
    if (time >= keys[count - 1].time) return keys[count - 1].value;
    for (u32 i = 0; i + 1 < count; ++i) {
        const HrAnimKeyVec3* a = &keys[i];
        const HrAnimKeyVec3* b = &keys[i + 1];
        if (time >= a->time && time <= b->time) {
            float span = b->time - a->time;
            float t = span > 1e-6f ? (time - a->time) / span : 0.0f;
            return v3_lerp(a->value, b->value, t);
        }
    }
    return keys[count - 1].value;
}

static Quat sample_quat(const HrAnimKeyQuat* keys, u32 count, float time, Quat fallback) {
    if (count == 0 || !keys) return fallback;
    if (time <= keys[0].time) return keys[0].value;
    if (time >= keys[count - 1].time) return keys[count - 1].value;
    for (u32 i = 0; i + 1 < count; ++i) {
        const HrAnimKeyQuat* a = &keys[i];
        const HrAnimKeyQuat* b = &keys[i + 1];
        if (time >= a->time && time <= b->time) {
            float span = b->time - a->time;
            float t = span > 1e-6f ? (time - a->time) / span : 0.0f;
            return quat_slerp(a->value, b->value, t);
        }
    }
    return keys[count - 1].value;
}

bool hr_animation_sample(const HrAnimationClip* clip, float time, HrPose* out_pose, u32 joint_count) {
    if (!clip || !out_pose) return false;
    hr_pose_identity(out_pose, joint_count);
    float t = wrap_time(time, clip->duration, clip->looping);
    for (u32 i = 0; i < clip->track_count; ++i) {
        const HrAnimTrack* tr = &clip->tracks[i];
        if (tr->joint_index >= out_pose->joint_count) continue;
        u32 j = tr->joint_index;
        out_pose->translations[j] = sample_vec3(tr->positions, tr->position_count, t, out_pose->translations[j]);
        out_pose->rotations[j] = sample_quat(tr->rotations, tr->rotation_count, t, out_pose->rotations[j]);
        out_pose->scales[j] = sample_vec3(tr->scales, tr->scale_count, t, out_pose->scales[j]);
    }
    return true;
}

void hr_anim_controller_init(HrAnimController* c, const HrSkeleton* skel) {
    if (!c) return;
    memset(c, 0, sizeof(*c));
    c->skeleton = skel;
    c->speed = 1.0f;
}

void hr_anim_controller_play(HrAnimController* c, const HrAnimationClip* clip, float fade_duration) {
    if (!c) return;
    if (c->current == clip) return;
    if (fade_duration > 0.0f && c->current) {
        c->previous = c->current;
        c->prev_time = c->time;
        c->fade_duration = fade_duration;
        c->fade_elapsed = 0.0f;
    } else {
        c->previous = NULL;
        c->fade_duration = 0.0f;
        c->fade_elapsed = 0.0f;
    }
    c->current = clip;
    c->time = 0.0f;
}

void hr_anim_controller_update(HrAnimController* c, float dt) {
    if (!c) return;
    float adv = dt * (c->speed != 0.0f ? c->speed : 1.0f);
    if (c->current) c->time += adv;
    if (c->previous) {
        c->prev_time += adv;
        c->fade_elapsed += dt;
        if (c->fade_elapsed >= c->fade_duration) {
            c->previous = NULL;
            c->fade_duration = 0.0f;
            c->fade_elapsed = 0.0f;
        }
    }
}

void hr_anim_controller_pose(const HrAnimController* c, HrPose* out_pose) {
    if (!c || !out_pose) return;
    u32 joints = c->skeleton ? c->skeleton->joint_count : 0;
    hr_pose_identity(out_pose, joints);
    if (!c->current) return;
    HrPose curr;
    hr_animation_sample(c->current, c->time, &curr, joints);
    if (c->previous && c->fade_duration > 0.0f) {
        HrPose prev;
        hr_animation_sample(c->previous, c->prev_time, &prev, joints);
        float t = HR_CLAMP(c->fade_elapsed / c->fade_duration, 0.0f, 1.0f);
        hr_pose_lerp(&prev, &curr, t, out_pose);
    } else {
        *out_pose = curr;
    }
}

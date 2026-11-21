#include "horizon/animation.h"
#include "horizon/memory.h"

#include <math.h>

void hr_skeleton_init(HrSkeleton* s) {
    if (!s) return;
    memset(s, 0, sizeof(*s));
}

i32 hr_skeleton_add_joint(HrSkeleton* s, const char* name, i32 parent,
                          Mat4 inverse_bind) {
    if (!s || s->joint_count >= HR_ANIM_MAX_JOINTS) return -1;
    if (parent >= (i32)s->joint_count) return -1;
    u32 index = s->joint_count++;
    HrJoint* j = &s->joints[index];
    j->parent = parent;
    j->inverse_bind = inverse_bind;
    if (name) {
        snprintf(j->name, sizeof(j->name), "%s", name);
    }
    return (i32)index;
}

i32 hr_skeleton_find_joint(const HrSkeleton* s, const char* name) {
    if (!s || !name) return -1;
    for (u32 i = 0; i < s->joint_count; ++i)
        if (strcmp(s->joints[i].name, name) == 0) return (i32)i;
    return -1;
}

HrAnimationClip* hr_animation_clip_create(const char* name, float duration, bool loop) {
    HrAnimationClip* c = (HrAnimationClip*)hr_calloc(1, sizeof(HrAnimationClip));
    if (!c) return NULL;
    if (name) snprintf(c->name, sizeof(c->name), "%s", name);
    c->duration = duration > 0.0f ? duration : 1.0f;
    c->loop = loop;
    return c;
}

void hr_animation_clip_destroy(HrAnimationClip* clip) {
    if (!clip) return;
    for (u32 i = 0; i < clip->track_count; ++i) hr_free(clip->tracks[i].keys);
    hr_free(clip->tracks);
    hr_free(clip);
}

HrAnimTrack* hr_animation_clip_add_track(HrAnimationClip* clip, i32 joint) {
    if (!clip || joint < 0 || joint >= HR_ANIM_MAX_JOINTS) return NULL;
    for (u32 i = 0; i < clip->track_count; ++i)
        if (clip->tracks[i].joint == joint) return &clip->tracks[i];

    if (clip->track_count == clip->track_cap) {
        u32 cap = clip->track_cap ? clip->track_cap * 2u : 4u;
        HrAnimTrack* nt = (HrAnimTrack*)hr_realloc(clip->tracks,
                                                   (usize)cap * sizeof(HrAnimTrack));
        if (!nt) return NULL;
        clip->tracks = nt;
        clip->track_cap = cap;
    }
    HrAnimTrack* t = &clip->tracks[clip->track_count++];
    memset(t, 0, sizeof(*t));
    t->joint = joint;
    return t;
}

void hr_anim_track_add_key(HrAnimTrack* track, float time,
                           Vec3 translation, Quat rotation, Vec3 scale) {
    if (!track) return;
    if (track->key_count == track->key_cap) {
        u32 cap = track->key_cap ? track->key_cap * 2u : 8u;
        HrAnimKey* nk = (HrAnimKey*)hr_realloc(track->keys,
                                               (usize)cap * sizeof(HrAnimKey));
        if (!nk) return;
        track->keys = nk;
        track->key_cap = cap;
    }
    u32 pos = track->key_count;
    while (pos > 0 && track->keys[pos - 1u].time > time) pos--;
    if (pos < track->key_count) {
        memmove(&track->keys[pos + 1u], &track->keys[pos],
                (usize)(track->key_count - pos) * sizeof(HrAnimKey));
    }
    HrAnimKey* k = &track->keys[pos];
    k->time = time;
    k->translation = translation;
    k->rotation = quat_norm(rotation);
    k->scale = scale;
    track->key_count++;
}

static void sample_track(const HrAnimTrack* track, float time, bool loop,
                         float duration, Vec3* t, Quat* r, Vec3* s) {
    u32 n = track->key_count;
    if (n == 0) return;
    if (loop && duration > 0.0f) {
        time = fmodf(time, duration);
        if (time < 0.0f) time += duration;
    }
    if (time <= track->keys[0].time) {
        *t = track->keys[0].translation;
        *r = track->keys[0].rotation;
        *s = track->keys[0].scale;
        return;
    }
    if (time >= track->keys[n - 1u].time) {
        *t = track->keys[n - 1u].translation;
        *r = track->keys[n - 1u].rotation;
        *s = track->keys[n - 1u].scale;
        return;
    }
    u32 i = 0;
    while (i + 1u < n && track->keys[i + 1u].time <= time) i++;
    const HrAnimKey* a = &track->keys[i];
    const HrAnimKey* b = &track->keys[i + 1u];
    float span = b->time - a->time;
    float u = span > 1e-8f ? (time - a->time) / span : 0.0f;
    *t = v3_lerp(a->translation, b->translation, u);
    *r = quat_slerp(a->rotation, b->rotation, u);
    *s = v3_lerp(a->scale, b->scale, u);
}

bool hr_animation_sample(const HrAnimationClip* clip, float time, HrPose* out) {
    if (!clip || !out) return false;
    u32 max_joint = 0;
    bool any = false;
    for (u32 i = 0; i < clip->track_count; ++i) {
        i32 j = clip->tracks[i].joint;
        if (j < 0) continue;
        if (!any || (u32)j > max_joint) max_joint = (u32)j;
        any = true;
    }
    if (!any) {
        out->joint_count = 0;
        return false;
    }
    u32 count = max_joint + 1u;
    for (u32 i = 0; i < count; ++i) {
        out->translation[i] = v3(0, 0, 0);
        out->rotation[i] = quat_identity();
        out->scale[i] = v3s(1.0f);
    }
    out->joint_count = count;
    for (u32 i = 0; i < clip->track_count; ++i) {
        const HrAnimTrack* tr = &clip->tracks[i];
        if (tr->joint < 0 || (u32)tr->joint >= count) continue;
        sample_track(tr, time, clip->loop, clip->duration,
                     &out->translation[tr->joint],
                     &out->rotation[tr->joint],
                     &out->scale[tr->joint]);
    }
    return true;
}

void hr_pose_bind(const HrSkeleton* skeleton, HrPose* out) {
    if (!out) return;
    u32 count = skeleton ? skeleton->joint_count : 0;
    for (u32 i = 0; i < count; ++i) {
        out->translation[i] = v3(0, 0, 0);
        out->rotation[i] = quat_identity();
        out->scale[i] = v3s(1.0f);
    }
    out->joint_count = count;
}

void hr_pose_blend(const HrPose* a, const HrPose* b, float t, HrPose* out) {
    if (!a || !b || !out) return;
    float u = hr_clamp01(t);
    u32 count = HR_MAX(a->joint_count, b->joint_count);
    if (count > HR_ANIM_MAX_JOINTS) count = HR_ANIM_MAX_JOINTS;
    for (u32 i = 0; i < count; ++i) {
        Vec3 ta = i < a->joint_count ? a->translation[i] : v3(0, 0, 0);
        Vec3 tb = i < b->joint_count ? b->translation[i] : v3(0, 0, 0);
        Quat ra = i < a->joint_count ? a->rotation[i] : quat_identity();
        Quat rb = i < b->joint_count ? b->rotation[i] : quat_identity();
        Vec3 sa = i < a->joint_count ? a->scale[i] : v3s(1.0f);
        Vec3 sb = i < b->joint_count ? b->scale[i] : v3s(1.0f);
        out->translation[i] = v3_lerp(ta, tb, u);
        out->rotation[i] = quat_slerp(ra, rb, u);
        out->scale[i] = v3_lerp(sa, sb, u);
    }
    out->joint_count = count;
}

void hr_pose_compute_matrices(const HrSkeleton* skeleton, const HrPose* pose,
                              Mat4* out_local, Mat4* out_world) {
    if (!skeleton || !pose) return;
    u32 count = HR_MIN(skeleton->joint_count, pose->joint_count);
    Mat4 local[HR_ANIM_MAX_JOINTS];
    for (u32 i = 0; i < count; ++i) {
        local[i] = mat4_compose(pose->translation[i], pose->rotation[i],
                                pose->scale[i]);
        if (out_local) out_local[i] = local[i];
        if (out_world) {
            i32 p = skeleton->joints[i].parent;
            out_world[i] = (p >= 0 && (u32)p < i)
                               ? mat4_mul(out_world[p], local[i])
                               : local[i];
        }
    }
}

void hr_pose_palette(const HrSkeleton* skeleton, const HrPose* pose,
                     Mat4* out_palette) {
    if (!skeleton || !pose || !out_palette) return;
    u32 count = HR_MIN(skeleton->joint_count, pose->joint_count);
    Mat4 world[HR_ANIM_MAX_JOINTS];
    hr_pose_compute_matrices(skeleton, pose, NULL, world);
    for (u32 i = 0; i < count; ++i)
        out_palette[i] = mat4_mul(world[i], skeleton->joints[i].inverse_bind);
}

void hr_animator_init(HrAnimator* anim, const HrSkeleton* skeleton) {
    if (!anim) return;
    memset(anim, 0, sizeof(*anim));
    anim->skeleton = skeleton;
    anim->speed = 1.0f;
    anim->blend = 1.0f;
    anim->paused = false;
}

void hr_animator_play(HrAnimator* anim, const HrAnimationClip* clip,
                      float fade_seconds) {
    if (!anim || !clip) return;
    if (anim->current == clip) return;
    anim->previous = anim->current;
    anim->previous_time = anim->current_time;
    anim->current = clip;
    anim->current_time = 0.0f;
    if (anim->previous && fade_seconds > 0.0f) {
        anim->blend = 0.0f;
        anim->blend_speed = 1.0f / fade_seconds;
    } else {
        anim->previous = NULL;
        anim->blend = 1.0f;
        anim->blend_speed = 0.0f;
    }
}

void hr_animator_update(HrAnimator* anim, float dt) {
    if (!anim || anim->paused || dt <= 0.0f) return;
    float step = dt * anim->speed;
    if (anim->current) {
        anim->current_time += step;
        if (anim->current->loop && anim->current->duration > 0.0f)
            anim->current_time = fmodf(anim->current_time, anim->current->duration);
    }
    if (anim->previous) {
        anim->previous_time += step;
        if (anim->previous->loop && anim->previous->duration > 0.0f)
            anim->previous_time = fmodf(anim->previous_time, anim->previous->duration);
        anim->blend += anim->blend_speed * dt;
        if (anim->blend >= 1.0f) {
            anim->blend = 1.0f;
            anim->previous = NULL;
        }
    }
}

void hr_animator_pose(const HrAnimator* anim, HrPose* out) {
    if (!anim || !out) return;
    if (!anim->current) {
        hr_pose_bind(anim->skeleton, out);
        return;
    }
    HrPose cur;
    hr_animation_sample(anim->current, anim->current_time, &cur);
    if (anim->previous && anim->blend < 1.0f) {
        HrPose prev;
        hr_animation_sample(anim->previous, anim->previous_time, &prev);
        hr_pose_blend(&prev, &cur, anim->blend, out);
    } else {
        *out = cur;
    }
    if (anim->skeleton) out->joint_count = anim->skeleton->joint_count;
}

/* Horizon Engine - skeletal animation: skeleton, clips, blending, palettes. */
#ifndef HORIZON_ANIMATION_H
#define HORIZON_ANIMATION_H

#include "horizon/math.h"

#define HR_ANIM_MAX_JOINTS 128
#define HR_ANIM_NAME 32

typedef struct HrJoint {
    i32 parent;
    Mat4 inverse_bind;
    char name[HR_ANIM_NAME];
} HrJoint;

typedef struct HrSkeleton {
    HrJoint joints[HR_ANIM_MAX_JOINTS];
    u32 joint_count;
} HrSkeleton;

typedef struct HrAnimKey {
    float time;
    Vec3 translation;
    Quat rotation;
    Vec3 scale;
} HrAnimKey;

typedef struct HrAnimTrack {
    i32 joint;
    HrAnimKey* keys;
    u32 key_count;
    u32 key_cap;
} HrAnimTrack;

typedef struct HrAnimationClip {
    char name[48];
    float duration;
    bool loop;
    HrAnimTrack* tracks;
    u32 track_count;
    u32 track_cap;
} HrAnimationClip;

typedef struct HrPose {
    Vec3 translation[HR_ANIM_MAX_JOINTS];
    Quat rotation[HR_ANIM_MAX_JOINTS];
    Vec3 scale[HR_ANIM_MAX_JOINTS];
    u32 joint_count;
} HrPose;

typedef struct HrAnimator {
    const HrSkeleton* skeleton;
    const HrAnimationClip* current;
    const HrAnimationClip* previous;
    float current_time;
    float previous_time;
    float blend;        /* 0 = previous, 1 = current */
    float blend_speed;
    float speed;
    bool paused;
} HrAnimator;

void hr_skeleton_init(HrSkeleton* s);
i32  hr_skeleton_add_joint(HrSkeleton* s, const char* name, i32 parent,
                           Mat4 inverse_bind);
i32  hr_skeleton_find_joint(const HrSkeleton* s, const char* name);

HrAnimationClip* hr_animation_clip_create(const char* name, float duration, bool loop);
void hr_animation_clip_destroy(HrAnimationClip* clip);

/* Reallocating the track array invalidates previously returned pointers. */
HrAnimTrack* hr_animation_clip_add_track(HrAnimationClip* clip, i32 joint);
void hr_anim_track_add_key(HrAnimTrack* track, float time,
                           Vec3 translation, Quat rotation, Vec3 scale);

/* Samples local-space TRS.  Looping clips wrap the time; joint_count is set to
 * the highest animated joint + 1. */
bool hr_animation_sample(const HrAnimationClip* clip, float time, HrPose* out);

void hr_pose_bind(const HrSkeleton* skeleton, HrPose* out);
void hr_pose_blend(const HrPose* a, const HrPose* b, float t, HrPose* out);

/* Parents must precede children (enforced by hr_skeleton_add_joint). */
void hr_pose_compute_matrices(const HrSkeleton* skeleton, const HrPose* pose,
                              Mat4* out_local, Mat4* out_world);
void hr_pose_palette(const HrSkeleton* skeleton, const HrPose* pose,
                     Mat4* out_palette);

void hr_animator_init(HrAnimator* anim, const HrSkeleton* skeleton);
void hr_animator_play(HrAnimator* anim, const HrAnimationClip* clip, float fade_seconds);
void hr_animator_update(HrAnimator* anim, float dt);
void hr_animator_pose(const HrAnimator* anim, HrPose* out);

#endif /* HORIZON_ANIMATION_H */

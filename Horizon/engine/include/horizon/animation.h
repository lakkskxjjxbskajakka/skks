/* Horizon Engine - skeletal animation: skeleton, pose, clips, sampling,
 * blending and a lightweight cross-fade controller. */
#ifndef HORIZON_ANIMATION_H
#define HORIZON_ANIMATION_H

#include "horizon/core.h"
#include "horizon/math.h"

#define HR_ANIM_MAX_JOINTS 128
#define HR_ANIM_MAX_TRACKS 128

typedef struct HrJoint {
    char name[64];
    i32 parent;
    Mat4 inverse_bind;
} HrJoint;

typedef struct HrSkeleton {
    HrJoint joints[HR_ANIM_MAX_JOINTS];
    u32 joint_count;
} HrSkeleton;

typedef struct HrPose {
    Vec3 translations[HR_ANIM_MAX_JOINTS];
    Quat rotations[HR_ANIM_MAX_JOINTS];
    Vec3 scales[HR_ANIM_MAX_JOINTS];
    u32 joint_count;
} HrPose;

typedef struct HrAnimKeyVec3 {
    float time;
    Vec3 value;
} HrAnimKeyVec3;

typedef struct HrAnimKeyQuat {
    float time;
    Quat value;
} HrAnimKeyQuat;

typedef struct HrAnimTrack {
    u32 joint_index;
    HrAnimKeyVec3* positions;
    u32 position_count;
    HrAnimKeyQuat* rotations;
    u32 rotation_count;
    HrAnimKeyVec3* scales;
    u32 scale_count;
} HrAnimTrack;

typedef struct HrAnimationClip {
    char name[64];
    float duration;
    bool looping;
    HrAnimTrack* tracks;
    u32 track_count;
    u32 track_capacity;
} HrAnimationClip;

typedef struct HrAnimController {
    const HrSkeleton* skeleton;
    const HrAnimationClip* current;
    const HrAnimationClip* previous;
    float time;
    float prev_time;
    float fade_duration;
    float fade_elapsed;
    float speed;
} HrAnimController;

void hr_skeleton_init(HrSkeleton* skel);
i32  hr_skeleton_add_joint(HrSkeleton* skel, const char* name, i32 parent, Mat4 inverse_bind);
i32  hr_skeleton_find_joint(const HrSkeleton* skel, const char* name);

void hr_pose_identity(HrPose* pose, u32 joint_count);
void hr_pose_bind(const HrSkeleton* skel, HrPose* out_pose);
void hr_pose_lerp(const HrPose* a, const HrPose* b, float t, HrPose* out);
void hr_pose_local_matrices(const HrPose* pose, Mat4* out_local, u32 count);
void hr_pose_palette(const HrSkeleton* skel, const HrPose* pose, Mat4* out_palette);

HrAnimationClip* hr_animation_clip_create(const char* name, float duration, bool looping);
void hr_animation_clip_destroy(HrAnimationClip* clip);
HrAnimTrack* hr_animation_clip_add_track(HrAnimationClip* clip, u32 joint_index);
bool hr_anim_track_add_position(HrAnimTrack* track, float time, Vec3 value);
bool hr_anim_track_add_rotation(HrAnimTrack* track, float time, Quat value);
bool hr_anim_track_add_scale(HrAnimTrack* track, float time, Vec3 value);

bool hr_animation_sample(const HrAnimationClip* clip, float time, HrPose* out_pose, u32 joint_count);

void hr_anim_controller_init(HrAnimController* c, const HrSkeleton* skel);
void hr_anim_controller_play(HrAnimController* c, const HrAnimationClip* clip, float fade_duration);
void hr_anim_controller_update(HrAnimController* c, float dt);
void hr_anim_controller_pose(const HrAnimController* c, HrPose* out_pose);

#endif /* HORIZON_ANIMATION_H */

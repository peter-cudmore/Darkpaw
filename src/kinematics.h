#include <cglm/cglm.h>
#include <stdbool.h>

#ifndef KINEMATICS_H
#define KINEMATICS_H

typedef unsigned int uint;

typedef enum {
    KineType_Link = 0,
    KineType_Rotation, 
    KineType_PinJoint,
    KineType_RevoluteJoint,
    KineType_MAX
} KineType;


typedef struct {
    vec3 offset; 
} KineLink;

typedef struct {
    vec3 normal;
    float angle;
} KineRotation;

typedef struct {
    vec3 normal;
    int joint_index;
} KinePinJoint;

typedef struct {
    vec3 normal;
    int joint_index;
} KineRevoluteJoint;

typedef struct {
    KineType type;
    union {
        KineLink link;
        KineRotation rotation;
        KinePinJoint pin_joint;
        KineRevoluteJoint revolute_joint;
    };
} KineChainElement;



typedef struct {
    int* indicies;
    uint32_t size;
    uint32_t max;
} KineIndexList;

// Defines a distance constraint such that
// ||(L_i...L_1L_0 - R_i...R_1R_0) p ||^2 = d^2 
// for any point p.

typedef struct {
    KineIndexList* left_chain;
    KineIndexList* right_chain;
    float distance;
} KineDistanceConstraint; 

typedef struct {
    KineIndexList angles;   // represents \sum a_iq^i  + remainder = 0
    float* coefficients;
    float remainder;
} KineLinearAngleEqConstraints;



typedef struct {
    KineChainElement* elements;
    uint max_elements;
    uint num_elements;
    
    KineIndexList joint_angles;    // 
    KineIndexList free_angles;
    
    // sequence of kinematic chain indicies for 
    KineIndexList end_effector;
    
    KineDistanceConstraint* dist_constraints;
    KineLinearAngleEqConstraints* angle_constraints;
    uint num_dist_constraints;
    uint num_angle_constraints;

} KineChain;
 


// Initialised a new kinematic chain.
void init_chain(KineChain* chain);

// Add constant elements to the chain.
int add_link(KineChain* chain, vec3 offset);
int add_fixed_rotation(KineChain* chain, vec3 normal, float angle);

// add a new joints and return the joint index.
int  add_new_pin_joint(KineChain* chain, vec3 normal);
int  add_new_revolute_joint(KineChain* chain, vec3 normal);

// add elements with the same joint value.
void add_exisiting_pin_joint(KineChain* chain, int joint_index);
void add_existing_revolute_joint(KineChain* chain, int joint_index);

// Adds a constraint such that for any point x || (A  - B)x || = d
void define_distance_constraint(KineChain* chain, KineIndexList indicies_a, KineIndexList indicies_b, float distance);

//
void define_end_effector(KineChain* chain, KineIndexList subchain);

// Inverts the kinemaic chain, turning it into the equivalent passive chain.
void active_to_passive(KineChain* source, KineChain* dest);


// Return true if the chain is properly defined - ie enog
bool is_valid(KineChain*);


// assumption: - length of angles are the same as the number of angles in the chain
bool transform_vector(KineChain* chain, float* angles, vec3 source, vec3 dest);


// assumption:  - out angles are allocated 
bool find_angles(KineChain* chain, vec3 displacement_from_rest, float* out_angles);


void free_chain(KineChain* chain);


#endif
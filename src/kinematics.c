#include "kinematics.h"
#include <stdbool.h>
#include <memory.h>
#include <assert.h>
#include <math.h>
// Initialised a new kinematic chain.

#define INITIAL_COUNT 8



void init_chain(KineChain* chain) {
    
    chain->elements = malloc(sizeof(KineChainElement) * INITIAL_COUNT);
    chain->max_elements = INITIAL_COUNT;
    chain->num_elements = 0;
    
    chain->free_angles.indicies = malloc(sizeof(int) * INITIAL_COUNT);
    chain->free_angles.max = INITIAL_COUNT;
    chain->free_angles.size = 0;

    chain->joint_angles.indicies = malloc(sizeof(int) * INITIAL_COUNT);
    chain->joint_angles.max = INITIAL_COUNT;
    chain->joint_angles.size = 0;

    chain->end_effector.indicies = NULL;
    chain->end_effector.max = 0;
    chain->end_effector.size = 0;

    chain->num_angle_constraints = 0;
    chain->num_dist_constraints = 0;
}

void grow_element_list(KineChain* chain) {
    chain->elements = realloc(chain->elements, 2 * chain->max_elements * sizeof(KineChainElement));
    assert(chain->elements);
    chain->max_elements *= 2;

}

void grow_index_list(KineIndexList* index_list) {
    index_list->indicies = realloc(index_list->indicies, 2 * sizeof(int) * index_list->max);
    assert(index_list->indicies);
    index_list->max *= 2;

}

int push_index_item(KineIndexList* list, int item) {
    
    if (list->max == list->size)
        grow_index_list(list);

    int id = list->size++;
    list->indicies[id] = item;
    return id;

}

void free_chain(KineChain* chain) {
    
}

// Add constant elements to the chain.
int add_link(KineChain* chain, vec3 offset) {
    assert(chain);
    if (chain->num_elements >= chain->max_elements) {
        grow_element_list(chain);
    }
    KineChainElement* this_element = chain->elements + chain->num_elements;
    this_element->type = KineType_Link;
    glm_vec3_copy(offset, this_element->link.offset);

    return chain->num_elements++;

}

int add_fixed_rotation(KineChain* chain, vec3 normal, float angle) {
    assert(chain);
       
    if (chain->num_elements >= chain->max_elements) {
        grow_element_list(chain);
    }

    KineChainElement* this_element = chain->elements + chain->num_elements;
    
    this_element->type = KineType_Rotation;
    this_element->rotation.angle = angle;
    glm_vec3_copy(normal, this_element->rotation.normal);

    return chain->num_elements++;;
}

// add a new joints and return the joint index.
int add_new_pin_joint(KineChain* chain, vec3 normal) {

}

int add_new_revolute_joint(KineChain* chain, vec3 normal) {
    
    assert(chain);
    if (chain->num_elements >= chain->max_elements) {
        grow_element_list(chain);
    }

    KineChainElement* this_element = chain->elements + chain->num_elements;
    int element_id = chain->num_elements++;
    
    this_element->type = KineType_RevoluteJoint;
    glm_vec3_copy(normal, this_element->revolute_joint.normal);
    
    this_element->revolute_joint.joint_index = push_index_item(&chain->joint_angles, element_id);
    return element_id;

}

// add elements with the same joint value.
void add_exisiting_pin_joint(KineChain* chain, int joint_index) { 

}

void add_existing_revolute_joint(KineChain* chain, int joint_index) {
}

// 
void define_distance_constraint(KineChain* chain, KineIndexList indicies_a, KineIndexList indicies_b, float distance) {

};


void define_end_effector(KineChain* chain, KineIndexList subchain) {

    assert(!chain->end_effector.indicies);
    
    chain->end_effector.indicies = malloc(sizeof(int) * subchain.size);
    chain->end_effector.size = subchain.size;
    chain->end_effector.max = subchain.size;

    memcpy(chain->end_effector.indicies, subchain.indicies, subchain.size * sizeof(int));

};

// Inverts the kinemaic chain, turning it into the equivalent passive chain.
void active_to_passive(KineChain* source, KineChain* dest) {

}


// Return true if the chain is properly defined - ie enog
bool is_valid(KineChain* chain) {
    bool result = (
        (chain->end_effector.indicies != NULL)
        && (chain->free_angles.size == chain->num_angle_constraints + chain->num_dist_constraints)
    
    );

    return result;
}


// assumption: - length of angles are the same as the number of angles in the chain
bool transform_vector(KineChain* chain, float* angles, vec3 source, vec3 dest) {

    if (!is_valid(chain)) { return false; }

    mat4 A;
    vec4 x = {0, 0, 0, 1};
    vec4 y;
    int id;
    KineIndexList* effector = &(chain->end_effector);
    glm_mat4_identity(A);
    
    glm_translate(A, source);


    for (int i = 0; i<  effector->size; i++) {
        KineChainElement *this_element = chain->elements + effector->indicies[i];
        switch (this_element->type)
        {
        case KineType_Link:
            glm_translate(A, this_element->link.offset);
            break;
        case KineType_Rotation:
            glm_rotate(A, this_element->rotation.angle, this_element->rotation.normal);
            break;
        case KineType_PinJoint:
            return false;   // must solve constraints first.
            break;
        case KineType_RevoluteJoint:
            id = this_element->revolute_joint.joint_index;
            glm_rotate(A, angles[id], this_element->revolute_joint.normal);
            break;
        default:
            fprintf(stderr, "Unknown chain element type %d\n", this_element->type);
            return false;
        }
    }

    glm_mat4_mulv(A, x, y);    
    glm_vec3_copy(y, dest);

    
    return true;
}


// assumption:  - out angles are allocated 
bool find_angles(KineChain* chain, vec3 vect_from_rest, float* out_angles) {
    return false;
}







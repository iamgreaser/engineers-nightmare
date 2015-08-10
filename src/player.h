#pragma once

#include <glm/glm.hpp>

#define EYE_OFFSET_Z    0.4
#define NUM_TOOL_SLOTS 5

struct tool;

struct player {
    float angle;
    float elev;
    glm::vec3 pos;
    glm::vec3 dir;  /* computed */
    glm::vec3 eye;  /* computed */
    glm::vec2 move;

    tool *tool_slots[NUM_TOOL_SLOTS];

    unsigned int selected_slot;

    bool jump;

    bool use;

    bool reset;

    bool crouch;
    bool crouch_end;

    bool gravity;

    bool use_tool;

    bool ui_dirty;

    /* used within physics to toggle gravity */
    bool disable_gravity;

    bool fire_projectile;
};


#include <stdio.h>
#include <SDL.h>

#ifndef _WIN32
#include <err.h> /* errx */
#else
#include "src/winerr.h"
#endif

#include <epoxy/gl.h>
#include <algorithm>
#include <functional>

#include <unordered_map>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "src/common.h"
#include "src/config.h"
#include "src/input.h"
#include "src/mesh.h"
#include "src/physics.h"
#include "src/player.h"
#include "src/shader.h"
#include "src/ship_space.h"
#include "src/text.h"
#include "src/textureset.h"
#include "src/tools.h"
#include "src/shader_params.h"
#include "src/light_field.h"
#include "src/projectiles.h"


#define APP_NAME    "Engineer's Nightmare"
#define DEFAULT_WIDTH   1024
#define DEFAULT_HEIGHT  768


#define WORLD_TEXTURE_DIMENSION     32
#define MAX_WORLD_TEXTURES          64

#define MOUSE_Y_LIMIT   1.54

bool exit_requested = false;

auto hfov = DEG2RAD(90.f);

en_settings game_settings;

struct wnd {
    SDL_Window *ptr;
    SDL_GLContext gl_ctx;
    int width;
    int height;
} wnd;

Uint32 last_frame_time = 0;

void GLAPIENTRY
gl_debug_callback(GLenum source __unused,
                  GLenum type __unused,
                  GLenum id __unused,
                  GLenum severity __unused,
                  GLsizei length __unused,
                  GLchar const *message,
                  void const *userParam __unused)
{
    printf("GL: %s\n", message);
}

sw_mesh *scaffold_sw;
sw_mesh *surfs_sw[6];
sw_mesh *projectile_sw;
GLuint simple_shader, unlit_shader, add_overlay_shader, remove_overlay_shader, ui_shader;
GLuint sky_shader;
shader_params<per_camera_params> *per_camera;
shader_params<per_object_params> *per_object;
texture_set *world_textures;
texture_set *skybox;
ship_space *ship;
player pl;
physics *phy;
unsigned char const *keys;
unsigned int mouse_buttons[input_mouse_buttons_count];
int mouse_axes[input_mouse_axes_count];
hw_mesh *scaffold_hw;
hw_mesh *surfs_hw[6];
hw_mesh *projectile_hw;
text_renderer *text;
light_field *light;
entity *use_entity = nullptr;


glm::ivec3
get_block_containing(glm::vec3 v) {
    int x = v.x; if (v.x < 0) x--;
    int y = v.y; if (v.y < 0) y--;
    int z = v.z; if (v.z < 0) z--;

    return glm::ivec3(x, y, z);
}

glm::mat4
mat_position(float x, float y, float z)
{
    return glm::translate(glm::mat4(1), glm::vec3(x, y, z));
}

glm::mat4
mat_position(glm::vec3 pos)
{
    return glm::translate(glm::mat4(1), pos);
}

glm::mat4
mat_block_face(int x, int y, int z, int face)
{
    switch (face) {
    case surface_zp:
        return glm::rotate(glm::translate(glm::mat4(1), glm::vec3(x, y + 1, z + 1)), (float)M_PI, glm::vec3(1.0f, 0.0f, 0.0f));
    case surface_zm:
        return glm::translate(glm::mat4(1), glm::vec3(x, y, z));
    case surface_xp:
        return glm::rotate(glm::translate(glm::mat4(1), glm::vec3(x + 1, y, z)), -(float)M_PI/2, glm::vec3(0.0f, 1.0f, 0.0f));
    case surface_xm:
        return glm::rotate(glm::translate(glm::mat4(1), glm::vec3(x, y, z + 1)), (float)M_PI/2, glm::vec3(0.0f, 1.0f, 0.0f));
    case surface_yp:
        return glm::rotate(glm::translate(glm::mat4(1), glm::vec3(x, y + 1, z)), (float)M_PI/2, glm::vec3(1.0f, 0.0f, 0.0f));
    case surface_ym:
        return glm::rotate(glm::translate(glm::mat4(1), glm::vec3(x, y, z + 1)), -(float)M_PI/2, glm::vec3(1.0f, 0.0f, 0.0f));

    default:
        return glm::mat4(1);    /* unreachable */
    }
}



struct entity_type
{
    sw_mesh *sw;
    hw_mesh *hw;
    char const *name;
    btTriangleMesh *phys_mesh;
    btCollisionShape *phys_shape;
    float add_air_amount;
    float max_air_pressure;
};


entity_type entity_types[3];


struct entity
{
    /* TODO: replace this completely, it's silly. */
    int x, y, z;
    entity_type *type;
    btRigidBody *phys_body;
    int face;
    glm::mat4 mat;

    entity(int x, int y, int z, entity_type *type, int face)
        : x(x), y(y), z(z), type(type), phys_body(nullptr), face(face) {
        mat = mat_block_face(x, y, z, face);

        build_static_physics_rb_mat(&mat, type->phys_shape, &phys_body);

        /* so that we can get back to the entity from a phys raycast */
        phys_body->setUserPointer(this);
    }

    ~entity() {
        teardown_static_physics_setup(nullptr, nullptr, &phys_body);
    }

    void use() {
        /* used by the player */
        printf("player using the %s at %d %d %d\n",
               type->name, x, y, z);
    }

    void tick() {
        if (type->add_air_amount <= 0) {
            /* TODO: components */
            return;
        }

        /* topo node containing the ent */
        topo_info *t = topo_find(ship->get_topo_info(x, y, z));
        /* zoneinfo attached */
        zone_info *z = ship->get_zone_info(t);
        if (!z) {
            /* if there wasnt a zone, make one. */
            z = ship->zones[t] = new zone_info(0);
        }

        /* add some air if we can, up to our pressure limit */
        float max_air = type->max_air_pressure * t->size;
        if (z->air_amount < max_air)
            z->air_amount = std::min(max_air, z->air_amount + type->add_air_amount);
    }
};


void
set_light_level(int x, int y, int z, int level)
{
    if (x < 0 || x >= 128) return;
    if (y < 0 || y >= 128) return;
    if (z < 0 || z >= 128) return;

    int p = x + y * 128 + z * 128 * 128;
    if (level < 0) level = 0;
    if (level > 255) level = 255;
    light->data[p] = level;
}


unsigned char
get_light_level(int x, int y, int z)
{
    if (x < 0 || x >= 128) return 0;
    if (y < 0 || y >= 128) return 0;
    if (z < 0 || z >= 128) return 0;

    return light->data[x + y*128 + z*128*128];
}


const int light_atten = 50;
/* as far as we can ever light from a light source */
const int max_light_prop = (255 + light_atten - 1) / light_atten;

bool need_lightfield_update = false;
glm::ivec3 lightfield_update_mins;
glm::ivec3 lightfield_update_maxs;


void
mark_lightfield_update(int x, int y, int z)
{
    glm::ivec3 center = glm::ivec3(x, y, z);
    glm::ivec3 half_extent = glm::ivec3(max_light_prop, max_light_prop, max_light_prop);
    if (need_lightfield_update) {
        lightfield_update_mins = center - half_extent;
        lightfield_update_maxs = center + half_extent;
    }
    else {
        lightfield_update_mins = glm::min(lightfield_update_mins,
                center - half_extent);
        lightfield_update_maxs = glm::max(lightfield_update_maxs,
                center + half_extent);
        need_lightfield_update = true;
    }
}


void
update_lightfield()
{
    if (!need_lightfield_update) {
        /* nothing to do here */
        return;
    }

    /* TODO: opt for case where we're JUST adding light -- no need to clear & rebuild */
    /* This is general enough to cope with occluders & lights being added and removed. */

    /* 1. remove all existing light in the box */
    for (int k = lightfield_update_mins.z; k <= lightfield_update_maxs.z; k++)
        for (int j = lightfield_update_mins.y; j <= lightfield_update_maxs.y; j++)
            for (int i = lightfield_update_mins.x; i <= lightfield_update_maxs.x; i++)
                set_light_level(i, j, k, 0);

    /* 2. inject sources. the box is guaranteed to be big enough for max propagation
     * for all sources we'll add here. */
    /* TODO: inject only required sources. */
    for (int k = ship->min_z; k <= ship->max_z; k++) {
        for (int j = ship->min_y; j <= ship->max_y; j++) {
            for (int i = ship->min_x; i <= ship->max_x; i++) {
                chunk *ch = ship->get_chunk(i, j, k);
                if (ch) {
                    for (auto e : ch->entities) {
                        /* TODO: only some entities should do this. */
                        if (e->type == &entity_types[2]) {
                            set_light_level(e->x, e->y, e->z, 255);
                        }
                    }
                }
            }
        }
    }

    /* 3. propagate max_light_prop times. this is guaranteed to be enough to cover
     * the sources' area of influence. */
    for (int pass = 0; pass < max_light_prop; pass++) {
        for (int k = lightfield_update_mins.z; k <= lightfield_update_maxs.z; k++) {
            for (int j = lightfield_update_mins.y; j <= lightfield_update_maxs.y; j++) {
                for (int i = lightfield_update_mins.x; i <= lightfield_update_maxs.x; i++) {
                    int level = get_light_level(i, j, k);

                    block *b = ship->get_block(i, j, k);
                    if (!b)
                        continue;

                    if (light_permeable(b->surfs[surface_xm]))
                        level = std::max(level, get_light_level(i - 1, j, k) - light_atten);
                    if (light_permeable(b->surfs[surface_xp]))
                        level = std::max(level, get_light_level(i + 1, j, k) - light_atten);

                    if (light_permeable(b->surfs[surface_ym]))
                        level = std::max(level, get_light_level(i, j - 1, k) - light_atten);
                    if (light_permeable(b->surfs[surface_yp]))
                        level = std::max(level, get_light_level(i, j + 1, k) - light_atten);

                    if (light_permeable(b->surfs[surface_zm]))
                        level = std::max(level, get_light_level(i, j, k - 1) - light_atten);
                    if (light_permeable(b->surfs[surface_zp]))
                        level = std::max(level, get_light_level(i, j, k + 1) - light_atten);

                    set_light_level(i, j, k, level);
                }
            }
        }
    }

    /* All done. */
    light->upload();
    need_lightfield_update = false;
}


struct game_state {
    virtual ~game_state() {}

    virtual void handle_input() = 0;
    virtual void update(float dt) = 0;
    virtual void rebuild_ui() = 0;

    static game_state *create_play_state();
    static game_state *create_menu_state();
    static game_state *create_menu_inventory_state();
    static game_state *create_menu_settings_state();
};


game_state *state = game_state::create_play_state();

void
set_game_state(game_state *s)
{
    if (state)
        delete state;

    state = s;
    pl.ui_dirty = true; /* state change always requires a ui rebuild. */
}

void
prepare_chunks()
{
    /* walk all the chunks -- TODO: only walk chunks that might contribute to the view */
    for (int k = ship->min_z; k <= ship->max_z; k++) {
        for (int j = ship->min_y; j <= ship->max_y; j++) {
            for (int i = ship->min_x; i <= ship->max_x; i++) {
                chunk *ch = ship->get_chunk(i, j, k);
                if (ch) {
                    ch->prepare_render(i, j, k);
                }
            }
        }
    }
}

void
init()
{
    printf("%s starting up.\n", APP_NAME);
    printf("OpenGL version: %.1f\n", epoxy_gl_version() / 10.0f);

    if (epoxy_gl_version() < 33) {
        errx(1, "At least OpenGL 3.3 is required\n");
    }

    /* Enable GL debug extension */
    if (!epoxy_has_gl_extension("GL_KHR_debug"))
        errx(1, "No support for GL debugging, life isn't worth it.\n");

    glEnable(GL_DEBUG_OUTPUT);
    glDebugMessageCallback(reinterpret_cast<GLDEBUGPROC>(&gl_debug_callback), nullptr);

    /* Check for ARB_texture_storage */
    if (!epoxy_has_gl_extension("GL_ARB_texture_storage"))
        errx(1, "No support for ARB_texture_storage\n");

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);         /* pointers given by other libs may not be aligned */
    glEnable(GL_DEPTH_TEST);

    mesher_init();

    projectile_sw = load_mesh("mesh/cube.obj");
    set_mesh_material(projectile_sw, 3);
    projectile_hw = upload_mesh(projectile_sw);

    scaffold_sw = load_mesh("mesh/initial_scaffold.obj");

    surfs_sw[surface_xp] = load_mesh("mesh/x_quad_p.obj");
    surfs_sw[surface_xm] = load_mesh("mesh/x_quad.obj");
    surfs_sw[surface_yp] = load_mesh("mesh/y_quad_p.obj");
    surfs_sw[surface_ym] = load_mesh("mesh/y_quad.obj");
    surfs_sw[surface_zp] = load_mesh("mesh/z_quad_p.obj");
    surfs_sw[surface_zm] = load_mesh("mesh/z_quad.obj");

    for (int i = 0; i < 6; i++)
        surfs_hw[i] = upload_mesh(surfs_sw[i]);

    entity_types[0].sw = load_mesh("mesh/frobnicator.obj");
    set_mesh_material(entity_types[0].sw, 3);
    entity_types[0].hw = upload_mesh(entity_types[0].sw);
    entity_types[0].name = "Frobnicator";
    entity_types[0].add_air_amount = 0.1f;
    entity_types[0].max_air_pressure = 1.0f;
    build_static_physics_mesh(entity_types[0].sw, &entity_types[0].phys_mesh, &entity_types[0].phys_shape);

    entity_types[1].sw = load_mesh("mesh/panel_4x4.obj");
    set_mesh_material(entity_types[1].sw, 7);
    entity_types[1].hw = upload_mesh(entity_types[1].sw);
    entity_types[1].name = "Display Panel (4x4)";
    entity_types[1].add_air_amount = 0.0f;
    entity_types[1].max_air_pressure = 0.0f;
    build_static_physics_mesh(entity_types[1].sw, &entity_types[1].phys_mesh, &entity_types[1].phys_shape);

    entity_types[2].sw = load_mesh("mesh/panel_4x4.obj");
    set_mesh_material(entity_types[2].sw, 8);
    entity_types[2].hw = upload_mesh(entity_types[2].sw);
    entity_types[2].name = "Light (4x4)";
    entity_types[2].add_air_amount = 0.0f;
    entity_types[2].max_air_pressure = 0.0f;
    build_static_physics_mesh(entity_types[2].sw, &entity_types[2].phys_mesh, &entity_types[2].phys_shape);

    simple_shader = load_shader("shaders/simple.vert", "shaders/simple.frag");
    unlit_shader = load_shader("shaders/simple.vert", "shaders/unlit.frag");
    add_overlay_shader = load_shader("shaders/add_overlay.vert", "shaders/unlit.frag");
    remove_overlay_shader = load_shader("shaders/remove_overlay.vert", "shaders/unlit.frag");
    ui_shader = load_shader("shaders/ui.vert", "shaders/ui.frag");
    sky_shader = load_shader("shaders/sky.vert", "shaders/sky.frag");

    scaffold_hw = upload_mesh(scaffold_sw);         /* needed for overlay */

    glUseProgram(simple_shader);

    per_camera = new shader_params<per_camera_params>;
    per_object = new shader_params<per_object_params>;

    per_object->val.world_matrix = glm::mat4(1);    /* identity */

    per_camera->bind(0);
    per_object->bind(1);

    world_textures = new texture_set(GL_TEXTURE_2D_ARRAY, WORLD_TEXTURE_DIMENSION, MAX_WORLD_TEXTURES);
    world_textures->load(0, "textures/white.png");
    world_textures->load(1, "textures/scaffold.png");
    world_textures->load(2, "textures/plate.png");
    world_textures->load(3, "textures/frobnicator.png");
    world_textures->load(4, "textures/grate.png");
    world_textures->load(5, "textures/red.png");
    world_textures->load(6, "textures/glass.png");
    world_textures->load(7, "textures/display.png");
    world_textures->load(8, "textures/light.png");

    skybox = new texture_set(GL_TEXTURE_CUBE_MAP_ARRAY, 2048, 6);
    skybox->load(0, "textures/sky_right1.png");
    skybox->load(1, "textures/sky_left2.png");
    skybox->load(2, "textures/sky_top3.png");
    skybox->load(3, "textures/sky_bottom4.png");
    skybox->load(4, "textures/sky_front5.png");
    skybox->load(5, "textures/sky_back6.png");

    ship = ship_space::mock_ship_space();
    if( ! ship )
        errx(1, "Ship_space::mock_ship_space failed\n");

    ship->rebuild_topology();

    printf("Ship is %u chunks, %d..%d %d..%d %d..%d\n",
            (unsigned) ship->chunks.size(),
            ship->min_x, ship->max_x,
            ship->min_y, ship->max_y,
            ship->min_z, ship->max_z);

    game_settings = load_settings(en_config_base);
    en_settings user_settings = load_settings(en_config_user);
    game_settings.merge_with(user_settings);

    pl.angle = 0;
    pl.elev = 0;
    pl.pos = glm::vec3(3,2,2);
    pl.active_tool = nullptr;
    pl.selected_slot = 1;
    pl.ui_dirty = true;
    pl.disable_gravity = false;

    phy = new physics(&pl);

    glEnable(GL_CULL_FACE);
    glFrontFace(GL_CW);

    text = new text_renderer("fonts/pixelmix.ttf", 16);

    printf("World vertex size: %lu bytes\n", sizeof(vertex));

    light = new light_field();
    light->bind(1);

    /* put some crap in the lightfield */
    memset(light->data, 0, sizeof(light->data));
    light->upload();

    /* prepare the chunks -- this populates the physics data */
    prepare_chunks();
}


void
resize(int width, int height)
{
    /* TODO: resize offscreen (but screen-sized) surfaces, etc. */
    glViewport(0, 0, width, height);
    wnd.width = width;
    wnd.height = height;
    printf("Resized to %dx%d\n", width, height);
}


void
remove_ents_from_surface(int x, int y, int z, int face)
{
    chunk *ch = ship->get_chunk_containing(x, y, z);
    for (auto it = ch->entities.begin(); it != ch->entities.end(); /* */) {
        entity *e = *it;
        if (e->x == x && e->y == y && e->z == z && e->face == face) {
            delete e;
            it = ch->entities.erase(it);
        }
        else {
            it++;
        }
    }

    block *bl = ship->get_block(x, y, z);
    assert(bl);

    bl->surf_space[face] = 0;   /* we've popped *everything* off, it must be empty now */

    if (face == surface_zm) {

        if (bl->type == block_entity)
            bl->type = block_empty;

        ch->render_chunk.valid = false;
    }
}


struct add_block_entity_tool : tool
{
    entity_type *type;

    explicit add_block_entity_tool(entity_type *type) : type(type) {}

    bool can_use(raycast_info *rc) {
        if (!rc->hit || rc->inside)
            return false;

        /* don't allow placements that would cause the player to end up inside the ent and get stuck */
        glm::ivec3 pos(rc->px, rc->py, rc->pz);
        if (pos == get_block_containing(pl.eye) ||
            pos == get_block_containing(pl.pos))
            return false;

        block *bl = ship->get_block(rc->px, rc->py, rc->pz);

        if (bl) {
            /* check for surface ents that would conflict */
            for (int face = 0; face < face_count; face++)
                if (bl->surf_space[face])
                    return false;
        }

        /* block ents can only be placed in empty space, on a scaffold */
        return bl && rc->block->type == block_support;
    }

    void use(raycast_info *rc) override {
        if (!can_use(rc))
            return;

        /* dirty the chunk -- TODO: do we really have to do this when changing a cell from
         * empty -> entity? */
        chunk *ch = ship->get_chunk_containing(rc->px, rc->py, rc->pz);
        ch->render_chunk.valid = false;
        ch->entities.push_back(
            new entity(rc->px, rc->py, rc->pz, type, surface_zm)
            );

        block *bl = ship->get_block(rc->px, rc->py, rc->pz);
        bl->type = block_entity;

        /* consume ALL the space on the surfaces */
        for (int face = 0; face < face_count; face++)
            bl->surf_space[face] = ~0;
    }

    void preview(raycast_info *rc) override {
        if (!can_use(rc))
            return;

        per_object->val.world_matrix = mat_position(rc->px, rc->py, rc->pz);
        per_object->upload();

        draw_mesh(type->hw);

        /* draw a block overlay as well around the frobnicator */
        glUseProgram(add_overlay_shader);
        draw_mesh(scaffold_hw);
        glUseProgram(simple_shader);
    }

    void get_description(char *str) override {
        sprintf(str, "Place %s", type->name);
    }
};


struct add_surface_entity_tool : tool
{
    entity_type *type;

    add_surface_entity_tool(entity_type *type) : type(type) {}

    bool can_use(raycast_info *rc) {
        if (!rc->hit)
            return false;

        block *bl = rc->block;

        int index = normal_to_surface_index(rc);

        if (bl->surfs[index] == surface_none)
            return false;

        block *other_side = ship->get_block(rc->px, rc->py, rc->pz);
        unsigned short required_space = ~0; /* TODO: make this a prop of the type + subblock placement */

        if (other_side->surf_space[index ^ 1] & required_space) {
            /* no room on the surface */
            return false;
        }

        return true;
    }

    void use(raycast_info *rc) override {
        if (!can_use(rc))
            return;

        int index = normal_to_surface_index(rc);

        block *other_side = ship->get_block(rc->px, rc->py, rc->pz);
        unsigned short required_space = ~0; /* TODO: make this a prop of the type + subblock placement */

        chunk *ch = ship->get_chunk_containing(rc->px, rc->py, rc->pz);
        /* the chunk we're placing into is guaranteed to exist, because there's
         * a surface facing into it */
        assert(ch);
        ch->entities.push_back(
            new entity(rc->px, rc->py, rc->pz, type, index ^ 1)
            );

        /* take the space. */
        other_side->surf_space[index ^ 1] |= required_space;

        /* mark lighting for rebuild around this point */
        mark_lightfield_update(rc->px, rc->py, rc->pz);
    }

    void preview(raycast_info *rc) override {
        if (!can_use(rc))
            return;

        int index = normal_to_surface_index(rc);

        per_object->val.world_matrix = mat_block_face(rc->px, rc->py, rc->pz, index ^ 1);
        per_object->upload();

        draw_mesh(type->hw);

        /* draw a surface overlay here too */
        /* TODO: sub-block placement granularity -- will need a different overlay */
        per_object->val.world_matrix = mat_position(rc->x, rc->y, rc->z);
        per_object->upload();

        glUseProgram(add_overlay_shader);
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(-0.1, -0.1);
        draw_mesh(surfs_hw[index]);
        glPolygonOffset(0, 0);
        glDisable(GL_POLYGON_OFFSET_FILL);
        glUseProgram(simple_shader);
    }

    void get_description(char *str) override {
        sprintf(str, "Place %s on surface", type->name);
    }
};


struct remove_surface_entity_tool : tool
{
    bool can_use(raycast_info *rc) {
        return rc->hit;
    }

    void use(raycast_info *rc) override {
        if (!can_use(rc))
            return;

        int index = normal_to_surface_index(rc);
        remove_ents_from_surface(rc->px, rc->py, rc->pz, index^1);
        mark_lightfield_update(rc->px, rc->py, rc->pz);
    }

    void preview(raycast_info *rc) override {
        if (!can_use(rc))
            return;

        int index = normal_to_surface_index(rc);
        block *other_side = ship->get_block(rc->px, rc->py, rc->pz);

        if (!other_side->surf_space[index ^ 1]) {
            return;
        }

        per_object->val.world_matrix = mat_position(rc->x, rc->y, rc->z);
        per_object->upload();

        glUseProgram(remove_overlay_shader);
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(-0.1, -0.1);
        draw_mesh(surfs_hw[index]);
        glPolygonOffset(0, 0);
        glDisable(GL_POLYGON_OFFSET_FILL);
        glUseProgram(simple_shader);
    }

    void get_description(char *str) override {
        strcpy(str, "Remove surface entity");
    }
};

tool *tools[] = {
    tool::create_fire_projectile_tool(&pl),
    tool::create_add_block_tool(),
    tool::create_remove_block_tool(),
    tool::create_add_surface_tool(surface_wall),
    tool::create_add_surface_tool(surface_grate),
    tool::create_add_surface_tool(surface_glass),
    tool::create_remove_surface_tool(),
    new add_block_entity_tool(&entity_types[0]),
    new add_surface_entity_tool(&entity_types[1]),
    new add_surface_entity_tool(&entity_types[2]),
    new remove_surface_entity_tool(),
};


void
add_text_with_outline(char const *s, float x, float y, float r = 1, float g = 1, float b = 1)
{
    text->add(s, x - 2, y, 0, 0, 0);
    text->add(s, x + 2, y, 0, 0, 0);
    text->add(s, x, y - 2, 0, 0, 0);
    text->add(s, x, y + 2, 0, 0, 0);
    text->add(s, x, y, r, g, b);
}


struct time_accumulator
{
    float period;
    float accum;

    time_accumulator(float period) :
        period(period), accum(0.0f) {}

    void add(float dt) { accum += dt; }

    bool tick()
    {
        if (accum >= period) {
            accum -= period;
            return true;
        }
        else {
            return false;
        }
    }
};


time_accumulator main_tick_accum(1/15.0f);  /* 15Hz tick for game logic */
time_accumulator fast_tick_accum(1/60.0f);  /* 60Hz tick for motion */


void
update()
{
    Uint32 now = SDL_GetTicks();
    // frame delta time in seconds
    float dt = (now - last_frame_time) / 1000.f;
    last_frame_time = now;

    float depthClearValue = 1.0f;
    glClearBufferfv(GL_DEPTH, 0, &depthClearValue);

    pl.dir = glm::vec3(
            cosf(pl.angle) * cosf(pl.elev),
            sinf(pl.angle) * cosf(pl.elev),
            sinf(pl.elev)
            );

    pl.eye = pl.pos + glm::vec3(0, 0, EYE_OFFSET_Z);

    auto vfov = hfov * (float)wnd.height / wnd.width;

    glm::mat4 proj = glm::perspective(vfov, (float)wnd.width / wnd.height, 0.01f, 1000.0f);
    glm::mat4 view = glm::lookAt(pl.eye, pl.eye + pl.dir, glm::vec3(0, 0, 1));
    glm::mat4 centered_view = glm::lookAt(glm::vec3(0), pl.dir, glm::vec3(0, 0, 1));
    per_camera->val.view_proj_matrix = proj * view;
    per_camera->val.inv_centered_view_proj_matrix = glm::inverse(proj * centered_view);
    per_camera->upload();

    main_tick_accum.add(dt);
    fast_tick_accum.add(dt);

    /* this absolutely must run every frame */
    state->update(dt);

    /* things that can run at a pretty slow rate */
    while (main_tick_accum.tick()) {

        /* rebuild lighting if needed */
        update_lightfield();

        /* remove any air that someone managed to get into the outside */
        {
            topo_info *t = topo_find(&ship->outside_topo_info);
            zone_info *z = ship->get_zone_info(t);
            if (z) {
                /* try as hard as you like, you cannot fill space with your air system */
                z->air_amount = 0;
            }
        }

        /* allow the entities to tick */
        for (auto ch : ship->chunks) {
            for (auto e : ch.second->entities) {
                e->tick();
            }
        }

        /* HACK: dirty this every frame for now while debugging atmo */
        if (1 || pl.ui_dirty) {
            text->reset();
            state->rebuild_ui();
            text->upload();
            pl.ui_dirty = false;
        }
    }

    /* character controller tick: we'd LIKE to run this off the fast_tick_accum, but it has all kinds of
     * every-frame assumptions baked in (player impulse state, etc) */
    phy->tick_controller(dt);

    while (fast_tick_accum.tick()) {

        update_projectiles(fast_tick_accum.period);

        phy->tick(fast_tick_accum.period);

    }

    world_textures->bind(0);

    prepare_chunks();

    for (int k = ship->min_z; k <= ship->max_z; k++) {
        for (int j = ship->min_y; j <= ship->max_y; j++) {
            for (int i = ship->min_x; i <= ship->max_x; i++) {
                /* TODO: prepare all the matrices first, and do ONE upload */
                chunk *ch = ship->get_chunk(i, j, k);
                if (ch) {
                    per_object->val.world_matrix = mat_position(
                                (float)i * CHUNK_SIZE, (float)j * CHUNK_SIZE, (float)k * CHUNK_SIZE);
                    per_object->upload();
                    draw_mesh(ch->render_chunk.mesh);
                }
            }
        }
    }

    /* walk all the entities in the (visible) chunks */
    for (int k = ship->min_z; k <= ship->max_z; k++) {
        for (int j = ship->min_y; j <= ship->max_y; j++) {
            for (int i = ship->min_x; i <= ship->max_x; i++) {
                chunk *ch = ship->get_chunk(i, j, k);
                if (ch) {
                    for (auto e : ch->entities) {
                        /* TODO: batch these matrix uploads too! */
                        per_object->val.world_matrix = e->mat;
                        per_object->upload();

                        draw_mesh(e->type->hw);
                    }
                }
            }
        }
    }

    /* draw the projectiles */
    glUseProgram(simple_shader);
    draw_projectiles();

    /* draw the sky */
    glUseProgram(sky_shader);
    skybox->bind(0);
    glDepthMask(GL_FALSE);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glDepthMask(GL_TRUE);

    /* draw the ui */
    glDisable(GL_DEPTH_TEST);

    glUseProgram(ui_shader);
    text->draw();
    glUseProgram(simple_shader);

    glEnable(GL_DEPTH_TEST);
}


action const* get_input(en_action a) {
    return &game_settings.bindings.bindings[a];
}


struct play_state : game_state {

    void rebuild_ui() override {
        float w = 0;
        float h = 0;
        char buf[256];
        char buf2[512];

        {
            /* Tool name down the bottom */
            tool *t = pl.active_tool;

            if (t) {
                t->get_description(buf);
            }
            else {
                strcpy(buf, "(no tool)");
            }
        }

        add_text_with_outline(".", 0, 0);

        sprintf(buf2, "Left mouse button: %s", buf);
        text->measure(buf2, &w, &h);
        add_text_with_outline(buf2, -w/2, -400);

        /* Gravity state (temp) */
        w = 0; h = 0;
        sprintf(buf, "Gravity: %s (G to toggle)", pl.disable_gravity ? "OFF" : "ON");
        text->measure(buf, &w, &h);
        add_text_with_outline(buf, -w/2, -430);

        /* Use key affordance */
        if (use_entity) {
            sprintf(buf2, "(E) Use the %s", use_entity->type->name);
            w = 0; h = 0;
            text->measure(buf2, &w, &h);
            add_text_with_outline(buf2, -w/2, -200);
        }

        {
            /* Atmo status */
            glm::ivec3 eye_block = get_block_containing(pl.eye);

            topo_info *t = topo_find(ship->get_topo_info(eye_block.x, eye_block.y, eye_block.z));
            topo_info *outside = topo_find(&ship->outside_topo_info);
            zone_info *z = ship->get_zone_info(t);
            float pressure = z ? (z->air_amount / t->size) : 0.0f;

            if (t != outside) {
                sprintf(buf2, "[INSIDE %p %d %.1f atmo]", t, t->size, pressure);
            }
            else {
                sprintf(buf2, "[OUTSIDE %p %d %.1f atmo]", t, t->size, pressure);
            }

            w = 0; h = 0;
            text->measure(buf2, &w, &h);
            add_text_with_outline(buf2, -w/2, -100);

            w = 0; h = 0;
            sprintf(buf2, "full: %d fast-unify: %d fast-nosplit: %d false-split: %d",
                    ship->num_full_rebuilds,
                    ship->num_fast_unifys,
                    ship->num_fast_nosplits,
                    ship->num_false_splits);
            text->measure(buf2, &w, &h);
            add_text_with_outline(buf2, -w/2, -150);
        }
    }

    void update(float dt) override {
        tool *t = pl.active_tool;

        /* both tool use and overlays need the raycast itself */
        raycast_info rc;
        ship->raycast(pl.eye, pl.dir, &rc);

        /* tool use */
        if (pl.use_tool && t) {
            t->use(&rc);
        }

        /* interact with ents */
        entity *hit_ent = phys_raycast(pl.eye, pl.eye + 2.f * pl.dir,
                                       phy->ghostObj, phy->dynamicsWorld);

        if (hit_ent != use_entity) {
            use_entity = hit_ent;
            pl.ui_dirty = true;
        }

        if (pl.use && hit_ent) {
            hit_ent->use();
        }

        /* tool preview */
        if (rc.hit && t) {
            t->preview(&rc);
        }
    }

    void set_slot(int slot) {
        pl.selected_slot = slot;
        pl.ui_dirty = true;
    }

    void cycle_slot(int d) {
        auto num_tools = sizeof(tools) / sizeof(tools[0]);
        unsigned int cur_slot = pl.selected_slot;
        cur_slot = (cur_slot + num_tools + d) % num_tools;

        pl.selected_slot = cur_slot;
        pl.ui_dirty = true;
    }

    void handle_input() override {
        /* look */
        auto look_x     = get_input(action_look_x)->value;
        auto look_y     = get_input(action_look_y)->value;

        /* movement */
        auto moveX      = get_input(action_right)->active - get_input(action_left)->active;
        auto moveY      = get_input(action_forward)->active - get_input(action_back)->active;

        /* crouch */
        auto crouch     = get_input(action_crouch)->active;
        auto crouch_end = get_input(action_crouch)->just_inactive;

        /* momentary */
        auto jump       = get_input(action_jump)->just_active;
        auto reset      = get_input(action_reset)->just_active;
        auto use        = get_input(action_use)->just_active;
        auto slot1      = get_input(action_slot1)->just_active;
        auto slot2      = get_input(action_slot2)->just_active;
        auto slot3      = get_input(action_slot3)->just_active;
        auto slot4      = get_input(action_slot4)->just_active;
        auto slot5      = get_input(action_slot5)->just_active;
        auto slot6      = get_input(action_slot6)->just_active;
        auto slot7      = get_input(action_slot7)->just_active;
        auto slot8      = get_input(action_slot8)->just_active;
        auto slot9      = get_input(action_slot9)->just_active;
        auto slot0      = get_input(action_slot0)->just_active;
        auto gravity    = get_input(action_gravity)->just_active;
        auto use_tool   = get_input(action_use_tool)->just_active;
        auto next_tool  = get_input(action_tool_next)->just_active;
        auto prev_tool  = get_input(action_tool_prev)->just_active;

        /* persistent */

        float mouse_invert = game_settings.input.mouse_invert;

        pl.angle += game_settings.input.mouse_x_sensitivity * look_x;
        pl.elev += game_settings.input.mouse_y_sensitivity * mouse_invert * look_y;

        if (pl.elev < -MOUSE_Y_LIMIT)
            pl.elev = -MOUSE_Y_LIMIT;
        if (pl.elev > MOUSE_Y_LIMIT)
            pl.elev = MOUSE_Y_LIMIT;

        pl.move.x = moveX;
        pl.move.y = moveY;

        pl.jump       = jump;
        pl.crouch     = crouch;
        pl.reset      = reset;
        pl.crouch_end = crouch_end;
        pl.use        = use;
        pl.gravity    = gravity;
        pl.use_tool   = use_tool;

        // blech. Tool gets used below, then fire projectile gets hit here
        if (pl.fire_projectile) {
            spawn_projectile(pl.eye, pl.dir);
            pl.fire_projectile = false;
        }

        if (next_tool) {
            cycle_slot(1);
        }
        if (prev_tool) {
            cycle_slot(-1);
        }

        if (slot1) set_slot(1);
        if (slot2) set_slot(2);
        if (slot3) set_slot(3);
        if (slot4) set_slot(4);
        if (slot5) set_slot(5);
        if (slot6) set_slot(6);
        if (slot7) set_slot(7);
        if (slot8) set_slot(8);
        if (slot9) set_slot(9);
        if (slot0) set_slot(0);

        /* limit to unit vector */
        float len = glm::length(pl.move);
        if (len > 0.0f)
            pl.move = pl.move / len;

        if (get_input(action_menu)->just_active) {
            set_game_state(create_menu_state());
        }
    }
};


struct menu_state : game_state
{
    typedef std::pair<char const *, std::function<void()>> menu_item;
    std::vector<menu_item> items;
    int selected = 0;

    menu_state() : items() {
        items.push_back(menu_item("Resume Game", []{ set_game_state(create_play_state()); }));
        items.push_back(menu_item("Inventory", [] { set_game_state(create_menu_inventory_state()); }));
        items.push_back(menu_item("Settings", []{ set_game_state(create_menu_settings_state()); }));
        items.push_back(menu_item("Exit Game", []{ exit_requested = true; }));
    }

    void update(float dt) override {
    }

    void put_item_text(char *dest, char const *src, int index) {
        if (index == selected)
            sprintf(dest, "> %s <", src);
        else
            strcpy(dest, src);
    }

    void rebuild_ui() override {
        float w = 0;
        float h = 0;
        char buf[256];

        sprintf(buf, "Engineer's Nightmare");
        text->measure(buf, &w, &h);
        add_text_with_outline(buf, -w/2, 300);

        float y = 50;
        float dy = -100;

        for (auto it = items.begin(); it != items.end(); it++) {
            w = 0;
            h = 0;
            put_item_text(buf, it->first, it - items.begin());
            text->measure(buf, &w, &h);
            add_text_with_outline(buf, -w/2, y);
            y += dy;
        }
    }

    void handle_input() override {
        if (get_input(action_menu_confirm)->just_active) {
            items[selected].second();
        }

        if (get_input(action_menu_down)->just_active) {
            selected = (selected + 1) % items.size();
            pl.ui_dirty = true;
        }

        if (get_input(action_menu_up)->just_active) {
            selected = (selected + items.size() - 1) % items.size();
            pl.ui_dirty = true;
        }

        if (get_input(action_menu)->just_active) {
            set_game_state(create_play_state());
        }
    }
};

struct menu_inventory_state : game_state {
    typedef std::tuple<char const *, char const *, std::function<void()>> menu_item;
    std::vector<menu_item> items;
    int selected = 0;
    unsigned int num_tools = sizeof(tools) / sizeof(tools[0]);
    char tool_descs[sizeof(tools) / sizeof(tools[0])][256];

    menu_inventory_state() {
    }

    void update(float dt) override {
        items.clear();

        items.push_back(
            menu_item("Back", "",
                [] { set_game_state(create_menu_state()); }));

        items.push_back(
            menu_item("Resume", "",
                [] { set_game_state(create_play_state()); }));

        for (int i = 0; i < num_tools; ++i) {
            auto tool = tools[i];
            if (!tool)
                continue;
            tool->get_description(tool_descs[i]);
            auto active = pl.active_tool == tool;
            items.push_back(menu_item(active ? "*" : "", tool_descs[i],
                [this, tool] { set_active_tool(tool); }));
        }
    }

    void put_item_text(char *dest, char const *src, int index) {
        if (index == selected)
            sprintf(dest, "> %s <", src);
        else
            strcpy(dest, src);
    }

    void set_active_tool(tool *tool) {
        pl.active_tool = tool;
    }

    void rebuild_ui() override {
        float w = 0;
        float h = 0;
        char buf[256];
        char buf2[256];

        sprintf(buf, "Engineer's Nightmare");
        text->measure(buf, &w, &h);
        add_text_with_outline(buf, -w / 2, 300);

        float y = 50;
        float dy = -30;

        for (auto it = items.begin(); it != items.end(); ++it) {
            w = 0;
            h = 0;
            sprintf(buf2, "%s %s", std::get<0>(*it), std::get<1>(*it));
            put_item_text(buf, buf2, it - items.begin());
            text->measure(buf, &w, &h);
            add_text_with_outline(buf, -w / 2, y);
            y += dy;
        }
    }


    void handle_input() override {
        if (get_input(action_menu_confirm)->just_active) {
            std::get<2>(items[selected])();
            pl.ui_dirty = true;
        }

        if (get_input(action_menu_down)->just_active) {
            selected = (selected + 1) % items.size();
            pl.ui_dirty = true;
        }

        if (get_input(action_menu_up)->just_active) {
            selected = (selected + items.size() - 1) % items.size();
            pl.ui_dirty = true;
        }

        if (get_input(action_menu)->just_active) {
            set_game_state(create_play_state());
        }
    }
};

struct menu_settings_state : game_state
{
    typedef std::tuple<char const *, char const *, std::function<void()>> menu_item;
    std::vector<menu_item> items;
    int selected = 0;

    char const *on_text = "On";
    char const *off_text = "Off";

    char const *invert_mouse_text = "Invert Mouse: ";

    Uint32 mouse_invert_mi = 0;

    menu_settings_state() {
        mouse_invert_mi = items.size();
        items.push_back(menu_item(invert_mouse_text, "",
            []{ toggle_mouse_invert(); }));
            // ^^ Not real keen on requiring these to be static

        items.push_back(
            menu_item("Save Settings", "",
            []{ save_settings(game_settings); }));

        items.push_back(
            menu_item("Back", "",
            []{ set_game_state(create_menu_state()); }));
    }

    static void toggle_mouse_invert() {
    // ^^ Not real keen on requiring these to be static
        game_settings.input.mouse_invert *= -1;
    }

    void update(float dt) override {
    }

    void put_item_text(char *dest, char const *src, int index) {
        if (index == selected)
            sprintf(dest, "> %s <", src);
        else
            strcpy(dest, src);
    }

    void rebuild_ui() override {
        menu_item *invert_item = &items.at(mouse_invert_mi);
        std::get<1>(*invert_item) = game_settings.input.mouse_invert > 0 ? off_text : on_text;

        float w = 0;
        float h = 0;
        char buf[256];
        char buf2[256];

        sprintf(buf, "Engineer's Nightmare");
        text->measure(buf, &w, &h);
        add_text_with_outline(buf, -w / 2, 300);

        float y = 50;
        float dy = -100;

        for (auto it = items.begin(); it != items.end(); ++it) {
            w = 0;
            h = 0;
            sprintf(buf2, "%s%s", std::get<0>(*it), std::get<1>(*it));
            put_item_text(buf, buf2, it - items.begin());
            text->measure(buf, &w, &h);
            add_text_with_outline(buf, -w / 2, y);
            y += dy;
        }
    }


    void handle_input() override {
        if (get_input(action_menu_confirm)->just_active) {
            std::get<2>(items[selected])();
            pl.ui_dirty = true;
        }

        if (get_input(action_menu_down)->just_active) {
            selected = (selected + 1) % items.size();
            pl.ui_dirty = true;
        }

        if (get_input(action_menu_up)->just_active) {
            selected = (selected + items.size() - 1) % items.size();
            pl.ui_dirty = true;
        }

        if (get_input(action_menu)->just_active) {
            set_game_state(create_play_state());
        }
    }
};


game_state *game_state::create_play_state() { return new play_state; }
game_state *game_state::create_menu_state() { return new menu_state; }
game_state *game_state::create_menu_inventory_state() { return new menu_inventory_state; }
game_state *game_state::create_menu_settings_state() { return new menu_settings_state; }


void
handle_input()
{
    set_inputs(keys, mouse_buttons, mouse_axes, game_settings.bindings.bindings);
    state->handle_input();
}


void
run()
{
    for (;;) {
        auto sdl_buttons = SDL_GetRelativeMouseState(nullptr, nullptr);
        mouse_buttons[EN_MOUSE_BUTTON(input_mouse_left)]      = sdl_buttons & EN_SDL_BUTTON(input_mouse_left);
        mouse_buttons[EN_MOUSE_BUTTON(input_mouse_middle)]    = sdl_buttons & EN_SDL_BUTTON(input_mouse_middle);
        mouse_buttons[EN_MOUSE_BUTTON(input_mouse_right)]     = sdl_buttons & EN_SDL_BUTTON(input_mouse_right);
        mouse_buttons[EN_MOUSE_BUTTON(input_mouse_thumb1)]    = sdl_buttons & EN_SDL_BUTTON(input_mouse_thumb1);
        mouse_buttons[EN_MOUSE_BUTTON(input_mouse_thumb2)]    = sdl_buttons & EN_SDL_BUTTON(input_mouse_thumb2);
        mouse_buttons[EN_MOUSE_BUTTON(input_mouse_wheeldown)] = false;
        mouse_buttons[EN_MOUSE_BUTTON(input_mouse_wheelup)]   = false;

        mouse_axes[EN_MOUSE_AXIS(input_mouse_x)] = 0.f;
        mouse_axes[EN_MOUSE_AXIS(input_mouse_y)] = 0.f;

        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            switch (e.type) {
            case SDL_QUIT:
                printf("Quit event caught, shutting down.\n");
                return;

            case SDL_WINDOWEVENT:
                /* We MUST support resize events even if we
                 * don't really care about resizing, because a tiling
                 * WM isn't going to give us what we asked for anyway!
                 */
                if (e.window.event == SDL_WINDOWEVENT_RESIZED)
                    resize(e.window.data1, e.window.data2);
                break;

            case SDL_MOUSEMOTION:
                mouse_axes[EN_MOUSE_AXIS(input_mouse_x)] += e.motion.xrel;
                mouse_axes[EN_MOUSE_AXIS(input_mouse_y)] += e.motion.yrel;
                break;

            case SDL_MOUSEWHEEL:
                if (e.wheel.y != 0) {
                    e.wheel.y > 0
                        ? mouse_buttons[EN_MOUSE_BUTTON(input_mouse_wheelup)] = true
                        : mouse_buttons[EN_MOUSE_BUTTON(input_mouse_wheeldown)] = true;
                }
                break;
            }
        }

        /* SDL_PollEvent above has already pumped the input, so current key state is available */
        handle_input();

        update();

        SDL_GL_SwapWindow(wnd.ptr);

        if (exit_requested) return;
    }
}

int
main(int, char **)
{
    if (SDL_Init(SDL_INIT_VIDEO) < 0)
        errx(1, "Error initializing SDL: %s\n", SDL_GetError());

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

    wnd.ptr = SDL_CreateWindow(APP_NAME,
                               SDL_WINDOWPOS_CENTERED,
                               SDL_WINDOWPOS_CENTERED,
                               DEFAULT_WIDTH,
                               DEFAULT_HEIGHT,
                               SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);

    if (!wnd.ptr)
        errx(1, "Failed to create window.\n");

    wnd.gl_ctx = SDL_GL_CreateContext(wnd.ptr);

    SDL_SetRelativeMouseMode(SDL_TRUE);
    keys = SDL_GetKeyboardState(nullptr);

    resize(DEFAULT_WIDTH, DEFAULT_HEIGHT);

    init();

    run();

    return 0;
}

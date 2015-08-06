#include <epoxy/gl.h>

#include "chunk.h"
#include "mesh.h"
#include "physics.h"

#include <glm/glm.hpp>
#include <vector>       // HISSSSSSS
#include <btBulletDynamicsCommon.h>

/* TODO: sensible container for these things, once we have variants */
extern sw_mesh *scaffold_sw;
extern sw_mesh *surfs_sw[6];


static void
stamp_at_offset(std::vector<vertex> *verts, std::vector<unsigned> *indices,
                sw_mesh *src, glm::vec3 offset, int mat)
{
    unsigned index_base = verts->size();

    for (unsigned int i = 0; i < src->num_vertices; i++) {
        vertex v = src->verts[i];
        v.x += offset.x;
        v.y += offset.y;
        v.z += offset.z;
        v.mat = mat;
        verts->push_back(v);
    }

    for (unsigned int i = 0; i < src->num_indices; i++)
        indices->push_back(index_base + src->indices[i]);
}


extern physics *phy;


void
build_static_physics_rb(int x, int y, int z, btCollisionShape *shape, btRigidBody **rb)
{
    if (*rb) {
        /* We already have a rigid body set up; just swap out its collision shape. */
        (*rb)->setCollisionShape(shape);
    }
    else {
        /* Rigid body doesn't exist yet -- build one, along with all th motionstate junk */
        btDefaultMotionState *ms = new btDefaultMotionState(
            btTransform(btQuaternion(0, 0, 0, 1), btVector3(x, y, z)));
        btRigidBody::btRigidBodyConstructionInfo
                    ci(0, ms, shape, btVector3(0, 0, 0));
        *rb = new btRigidBody(ci);
        phy->dynamicsWorld->addRigidBody(*rb);
    }
}


void
build_static_physics_rb_mat(glm::mat4 *m, btCollisionShape *shape, btRigidBody **rb)
{
    if (*rb) {
        /* We already have a rigid body set up; just swap out its collision shape. */
        (*rb)->setCollisionShape(shape);
    }
    else {
        /* Rigid body doesn't exist yet -- build one, along with all th motionstate junk */
        btTransform t;
        t.setFromOpenGLMatrix((float *)m);
        btDefaultMotionState *ms = new btDefaultMotionState(t);
        btRigidBody::btRigidBodyConstructionInfo
                    ci(0, ms, shape, btVector3(0, 0, 0));
        *rb = new btRigidBody(ci);
        phy->dynamicsWorld->addRigidBody(*rb);
    }
}


void
build_static_physics_mesh(sw_mesh const * src, btTriangleMesh **mesh, btCollisionShape **shape)
{
    btTriangleMesh *phys = NULL;
    btCollisionShape *new_shape = NULL;

    if (src->num_indices) {
        /* If we have some content in our mesh, transfer it to bullet */
        phys = new btTriangleMesh();
        phys->preallocateVertices(src->num_vertices);
        phys->preallocateIndices(src->num_indices);

        for (auto x = src->indices; x < src->indices + src->num_indices; /* */) {
            vertex v1 = src->verts[*x++];
            vertex v2 = src->verts[*x++];
            vertex v3 = src->verts[*x++];

            phys->addTriangle(btVector3(v1.x, v1.y, v1.z),
                              btVector3(v2.x, v2.y, v2.z),
                              btVector3(v3.x, v3.y, v3.z));
        }

        new_shape = new btBvhTriangleMeshShape(phys, true, true);
    }
    else {
        /* Empty mesh, just provide an empty shape. A zero-size mesh provokes a segfault inside
         * bullet, so avoid that. */
        new_shape = new btEmptyShape();
    }

    /* Throw away any old objects we've replaced. */
    if (*shape)
        delete *shape;
    *shape = new_shape;

    if (*mesh)
        delete *mesh;
    *mesh = phys;
}


void
teardown_static_physics_setup(btTriangleMesh **mesh, btCollisionShape **shape, btRigidBody **rb)
{
    /* cleanly teardown a static physics object such that build_static_physics_setup() will
     * properly reconstruct everything */

    if (rb && *rb) {
        phy->dynamicsWorld->removeRigidBody(*rb);
        delete *rb;
        *rb = NULL;
    }

    if (shape && *shape) {
        delete *shape;
        *shape = NULL;
    }

    if (mesh && *mesh) {
        delete *mesh;
        *mesh = NULL;
    }
}


static int surface_type_to_material[256];

void
mesher_init()
{
    memset(surface_type_to_material, 0, sizeof(surface_type_to_material));
    surface_type_to_material[surface_none] = 0;
    surface_type_to_material[surface_wall] = 2;
    surface_type_to_material[surface_grate] = 4;
    surface_type_to_material[surface_glass] = 6;
}


void
emit_mesh_lump(std::vector<vertex> &verts, std::vector<unsigned> &indices, std::vector<hw_mesh *> &meshes)
{
    /* wrap the vectors in a temporary sw_mesh */
    sw_mesh m;
    m.verts = &verts[0];
    m.indices = &indices[0];
    m.num_vertices = verts.size();
    m.num_indices = indices.size();

    meshes.push_back(upload_mesh(&m));

    indices.clear();
    verts.clear();
}

#define MAX_MESH_LUMP_VERTICES  65536


void
chunk::prepare_render(int x, int y, int z)
{
    if (this->render_chunk.valid)
        return;     // nothing to do here.

    std::vector<vertex> verts;
    std::vector<unsigned> indices;

    std::vector<hw_mesh *> meshes;

    for (int k = 0; k < CHUNK_SIZE; k++)
        for (int j = 0; j < CHUNK_SIZE; j++)
            for (int i = 0; i < CHUNK_SIZE; i++) {
                block *b = this->blocks.get(i, j, k);

                if (b->type == block_support) {
                    // TODO: block detail, variants, types, surfaces
                    if (scaffold_sw->num_vertices + verts.size() > MAX_MESH_LUMP_VERTICES)
                        emit_mesh_lump(verts, indices, meshes);

                    stamp_at_offset(&verts, &indices, scaffold_sw, glm::vec3(i, j, k), 1);
                }

                for (int surf = 0; surf < 6; surf++) {
                    if (b->surfs[surf] != surface_none) {
                        if (surfs_sw[surf]->num_vertices > MAX_MESH_LUMP_VERTICES)
                            emit_mesh_lump(verts, indices, meshes);

                        stamp_at_offset(&verts, &indices, surfs_sw[surf], glm::vec3(i, j, k),
                                surface_type_to_material[b->surfs[surf]]);
                    }
                }
            }

    if (verts.size()) {
        emit_mesh_lump(verts, indices, meshes);
    }

    /* TODO: we can be a LOT smarter about reusing BO memory etc. */
    if (this->render_chunk.meshes) {
        for (auto i = 0u; i < this->render_chunk.num_meshes; i++) {
            free_mesh(this->render_chunk.meshes[i]);
            delete this->render_chunk.meshes[i];
        }

        delete [] this->render_chunk.meshes;
    }

    this->render_chunk.num_meshes = meshes.size();
    this->render_chunk.meshes = new hw_mesh * [meshes.size()];

    for (auto i = 0u; i < meshes.size(); i++) {
        this->render_chunk.meshes[i] = meshes[i];
    }

    this->render_chunk.valid = true;
}


void
chunk::prepare_phys(int x, int y, int z)
{
    if (this->render_chunk.phys_valid)
        return;     // nothing to do here.

    std::vector<vertex> verts;
    std::vector<unsigned> indices;

    for (int k = 0; k < CHUNK_SIZE; k++)
        for (int j = 0; j < CHUNK_SIZE; j++)
            for (int i = 0; i < CHUNK_SIZE; i++) {
                block *b = this->blocks.get(i, j, k);

                if (b->type == block_support) {
                    // TODO: block detail, variants, types, surfaces

                    stamp_at_offset(&verts, &indices, scaffold_sw, glm::vec3(i, j, k), 1);
                }

                for (int surf = 0; surf < 6; surf++) {
                    if (b->surfs[surf] != surface_none) {

                        stamp_at_offset(&verts, &indices, surfs_sw[surf], glm::vec3(i, j, k),
                                surface_type_to_material[b->surfs[surf]]);
                    }
                }
            }

    this->render_chunk.phys_valid = true;

    /* wrap the vectors in a temporary sw_mesh */
    sw_mesh m;
    m.verts = &verts[0];
    m.indices = &indices[0];
    m.num_vertices = verts.size();
    m.num_indices = indices.size();

    build_static_physics_mesh(&m,
                              &this->render_chunk.phys_mesh,
                              &this->render_chunk.phys_shape);

    build_static_physics_rb(x * CHUNK_SIZE,
                            y * CHUNK_SIZE,
                            z * CHUNK_SIZE,
                            this->render_chunk.phys_shape,
                            &this->render_chunk.phys_body);
}


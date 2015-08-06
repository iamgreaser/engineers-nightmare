#include <stdio.h>

#ifndef _WIN32
#include <err.h> /* errx */
#else
#include "winerr.h"
#endif

#include <epoxy/gl.h>

#include "common.h"
#include "mesh.h"

#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/config.h>

// HISSSSS
#include <vector>


sw_mesh *
load_mesh(char const *filename) {

    static aiPropertyStore* meshImportProps = 0;
    if (!meshImportProps) {
        /* FIXME: threading hazard if we want to do concurrent loading */
        meshImportProps = aiCreatePropertyStore();
        aiSetImportPropertyInteger(meshImportProps, AI_CONFIG_PP_RVC_FLAGS, aiComponent_NORMALS);
    }

    aiScene const *scene = aiImportFileExWithProperties(filename, aiProcess_Triangulate |
            aiProcess_MakeLeftHanded | aiProcess_GenNormals | aiProcess_OptimizeMeshes | aiProcess_RemoveComponent,
            NULL /* IO */,
            meshImportProps);

    if (!scene)
        errx(1, "Failed to load mesh %s", filename);

    printf("Mesh %s:\n", filename);
    printf("\tNumMeshes: %d\n", scene->mNumMeshes);

    std::vector<vertex> verts;
    std::vector<unsigned> indices;

    for (unsigned int i = 0; i < scene->mNumMeshes; i++) {
        aiMesh const *m = scene->mMeshes[i];
        if (!m->mFaces)
            errx(1, "Submesh %u is not indexed.\n", i);

        // when we have many submeshes we need to rebase the indices.
        // NOTE: we assume that all mesh origins coincide, so we're not applying transforms here
        int submesh_base = verts.size();

        for (unsigned int j = 0; j < m->mNumVertices; j++)
            verts.push_back(vertex(m->mVertices[j].x, m->mVertices[j].y, m->mVertices[j].z,
                        m->mNormals[j].x, m->mNormals[j].y, m->mNormals[j].z, 0 /* mat */));

        for (unsigned int j = 0; j < m->mNumFaces; j++) {
            if (m->mFaces[j].mNumIndices != 3)
                errx(1, "Submesh %u face %u isnt a tri (%u indices)\n", i, j, m->mFaces[j].mNumIndices);

            indices.push_back(m->mFaces[j].mIndices[0] + submesh_base);
            indices.push_back(m->mFaces[j].mIndices[1] + submesh_base);
            indices.push_back(m->mFaces[j].mIndices[2] + submesh_base);
        }
    }

    printf("\tAfter processing: %lu verts, %lu indices\n", verts.size(), indices.size());

    aiReleaseImport(scene);

    sw_mesh *ret = new sw_mesh;
    ret->num_vertices = verts.size();
    ret->num_indices = indices.size();
    ret->verts = new vertex[verts.size()];
    memcpy(ret->verts, &verts[0], sizeof(vertex) * verts.size());
    ret->indices = new unsigned int[indices.size()];
    memcpy(ret->indices, &indices[0], sizeof(unsigned) * indices.size());
    return ret;
}

hw_mesh *
upload_mesh(sw_mesh *mesh)
{
    hw_mesh *ret = new hw_mesh;

    glCreateVertexArrays(1, &ret->vao);

    /* Setup vertex format */
    glVertexArrayAttribFormat(ret->vao, 0, 3, GL_FLOAT, GL_FALSE, offsetof(vertex, x));
    glVertexArrayAttribIFormat(ret->vao, 1, 1, GL_UNSIGNED_SHORT, offsetof(vertex, mat));
    glVertexArrayAttribFormat(ret->vao, 2, 4, GL_INT_2_10_10_10_REV, GL_FALSE, offsetof(vertex, normal_packed));

    glVertexArrayAttribBinding(ret->vao, 0, 0);
    glVertexArrayAttribBinding(ret->vao, 1, 0);
    glVertexArrayAttribBinding(ret->vao, 2, 0);

    glEnableVertexArrayAttrib(ret->vao, 0);
    glEnableVertexArrayAttrib(ret->vao, 1);
    glEnableVertexArrayAttrib(ret->vao, 2);

    /* Setup actual vertex data */
    glCreateBuffers(1, &ret->vbo);
    glCreateBuffers(1, &ret->ibo);

    glNamedBufferData(ret->vbo, mesh->num_vertices * sizeof(vertex), mesh->verts, GL_STATIC_DRAW);
    glNamedBufferData(ret->ibo, mesh->num_indices * sizeof(unsigned), mesh->indices, GL_STATIC_DRAW);

    glVertexArrayElementBuffer(ret->vao, ret->ibo);
    glVertexArrayVertexBuffer(ret->vao, 0, ret->vbo, 0, sizeof(vertex));

    ret->num_indices = mesh->num_indices;

    printf("upload_mesh: %p num_indices=%d vram_size=%.1fKB\n", ret,
            ret->num_indices, (mesh->num_vertices * sizeof(vertex) + mesh->num_indices * sizeof(unsigned)) / 1024.0f);

    return ret;
}


void
draw_mesh(hw_mesh *m)
{
    glBindVertexArray(m->vao);
    glDrawElements(GL_TRIANGLES, m->num_indices, GL_UNSIGNED_INT, nullptr);
}


void
free_mesh(hw_mesh *m)
{
    /* TODO: try to reuse BO */
    glDeleteBuffers(1, &m->vbo);
    glDeleteBuffers(1, &m->ibo);
    glDeleteVertexArrays(1, &m->vao);

    printf("free_mesh: %p num_indices=%d\n", m, m->num_indices);
}


void
set_mesh_material(sw_mesh *m, int material)
{
    for (unsigned int i = 0; i < m->num_vertices; i++) {
        m->verts[i].mat = material;
    }
}

#include "text.h"

#include <algorithm>
#include <ft2build.h>
#include FT_FREETYPE_H
#include <stdlib.h>

#ifndef _WIN32
#include <err.h> /* errx */
#else
#include "winerr.h"
#endif


#define TEXT_ATLAS_WIDTH    512
#define TEXT_ATLAS_HEIGHT   512


texture_atlas::texture_atlas()
    : tex(0), x(0), y(0), h(0)
{
    glCreateTextures(GL_TEXTURE_2D, 1, &tex);
    glTextureStorage2D(tex, 1, GL_R8, TEXT_ATLAS_WIDTH, TEXT_ATLAS_HEIGHT);
    glTextureParameteri(tex, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTextureParameteri(tex, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    buf = new unsigned char[TEXT_ATLAS_WIDTH * TEXT_ATLAS_HEIGHT];
}


void
texture_atlas::add_bitmap(unsigned char *src, int pitch, unsigned width, unsigned height, int *out_x, int *out_y)
{
    /* overflow to next row */
    if (x + width > TEXT_ATLAS_WIDTH) {
        y += h;
        x = 0;
        h = 0;
    }

    /* adjust height of atlas row */
    h = std::max(h, height);

    unsigned char *dest = buf + x + y * TEXT_ATLAS_WIDTH;

    for (unsigned int r = 0; r < height; r++) {
        memcpy(dest, src, width);
        src += pitch;
        dest += TEXT_ATLAS_WIDTH;
    }

    *out_x = x;
    *out_y = y;

    x += width;
}


void
texture_atlas::upload()
{
    glTextureSubImage2D(tex, 0, 0, 0, TEXT_ATLAS_WIDTH, TEXT_ATLAS_HEIGHT, GL_RED, GL_UNSIGNED_BYTE, buf);
}


void
texture_atlas::bind(int texunit)
{
    glActiveTexture(GL_TEXTURE0 + texunit);
    glBindTexture(GL_TEXTURE_2D, tex);
}


text_renderer::text_renderer(char const *font, int size)
    : bo(0), bo_vertex_count(0), bo_capacity(0), vao(0), verts()
{
    /* load the font into metrics array + texture */
    atlas = new texture_atlas();

    FT_Library ft_library;
    if (FT_Init_FreeType(&ft_library))
        errx(1, "freetype init failed");

    FT_Face ft_face;
    if (FT_New_Face(ft_library, font, 0, &ft_face))
        errx(1, "failed to load font `%s`", font);

    FT_Set_Pixel_Sizes(ft_face, 0, size);

    for (int i = 0; i < 256; i++) {
        metrics *m = &ms[i];

        /* load one glyph */
        FT_Load_Char(ft_face, i, FT_LOAD_TARGET_NORMAL | FT_LOAD_RENDER);
        FT_Bitmap const *bitmap = &ft_face->glyph->bitmap;

        atlas->add_bitmap(bitmap->buffer, bitmap->pitch, bitmap->width, bitmap->rows,
            &m->x, &m->y);

        m->w = bitmap->width;
        m->h = bitmap->rows;

        m->xoffset = ft_face->glyph->bitmap_left;
        m->advance = ft_face->glyph->linearHoriAdvance / 65536.0f;   /* 16.16 -> float */
        m->yoffset = ft_face->glyph->bitmap_top;
    }

    atlas->upload();

    glCreateVertexArrays(1, &vao);

    /* setup vertex format */
    glVertexArrayAttribFormat(vao, 0, 2, GL_FLOAT, GL_FALSE, offsetof(text_vertex, x));
    glVertexArrayAttribFormat(vao, 1, 2, GL_FLOAT, GL_FALSE, offsetof(text_vertex, u));
    glVertexArrayAttribFormat(vao, 2, 3, GL_FLOAT, GL_FALSE, offsetof(text_vertex, r));

    glEnableVertexArrayAttrib(vao, 0);
    glEnableVertexArrayAttrib(vao, 1);
    glEnableVertexArrayAttrib(vao, 2);

    glVertexArrayAttribBinding(vao, 0, 0);
    glVertexArrayAttribBinding(vao, 1, 0);
    glVertexArrayAttribBinding(vao, 2, 0);

    /* create a bo to work with */
    glCreateBuffers(1, &bo);
    glVertexArrayVertexBuffer(vao, 0, bo, 0, sizeof(text_vertex));

    printf("Loaded font %s at size %d\n", font, size);
}


void
text_renderer::add(char const *str, float x, float y, float r, float g, float b)
{
    float xx = x;

    for (; *str; str++) {
        if (*str == '\n') {
            y -= 24;
            xx = x;
            continue;
        }

        metrics *m = &ms[(unsigned) *str];

        float u0 = m->x / (float)TEXT_ATLAS_WIDTH;
        float u1 = (m->x + m->w) / (float)TEXT_ATLAS_WIDTH;
        float v0 = m->y / (float)TEXT_ATLAS_HEIGHT;
        float v1 = (m->y + m->h) / (float)TEXT_ATLAS_HEIGHT;

        xx += m->xoffset;
        float yy = y + m->yoffset;

        text_vertex p0 = { xx, yy, u0, v0, r, g, b };
        text_vertex p1 = { xx + m->w, yy, u1, v0, r, g, b };
        text_vertex p2 = { xx + m->w, yy - m->h, u1, v1, r, g, b };
        text_vertex p3 = { xx, yy - m->h, u0, v1, r, g, b };

        verts.push_back(p0);
        verts.push_back(p1);
        verts.push_back(p2);

        verts.push_back(p0);
        verts.push_back(p2);
        verts.push_back(p3);

        xx += m->advance;
    }
}


void
text_renderer::measure(char const *str, float *x, float *y)
{
    float xx = *x;

    for (; *str; str++) {
        if (*str == '\n') {
            *x = std::max(*x, xx);
            xx = 0;
            continue;
        }

        metrics *m = &ms[(unsigned) *str];

        xx += m->xoffset + m->advance;
    }

    *x = std::max(*x, xx);
}


void
text_renderer::upload()
{
    /* Exact-fit is pretty lousy as a growth strategy, but oh well */
    if (verts.size() > bo_capacity) {
        glNamedBufferData(bo, verts.size() * sizeof(text_vertex), &verts[0], GL_STREAM_DRAW);
        bo_capacity = verts.size();
        bo_vertex_count = verts.size();
    }
    else {
        glNamedBufferSubData(bo, 0, verts.size() * sizeof(text_vertex), &verts[0]);
        bo_vertex_count = verts.size();
    }
}


void
text_renderer::reset()
{
    verts.clear();
}


void
text_renderer::draw()
{
    glBindVertexArray(vao);
    atlas->bind(0);
    glDrawArrays(GL_TRIANGLES, 0, bo_vertex_count);
}

#include <stdio.h>

#ifndef _WIN32
#include <err.h> /* errx */
#else
#include "winerr.h"
#endif

#include <epoxy/gl.h>
#include <SDL.h>
#include <SDL_image.h>

#include "common.h"
#include "textureset.h"


texture_set::texture_set(GLenum target, int dim, int array_size)
    : texobj(0), dim(dim), array_size(array_size), target(target) {
    glCreateTextures(target, 1, &texobj);
    glTextureStorage3D(texobj,
            1,   /* no mips! I WANT YOUR EYES TO BLEED -- todo, fix this. */
            GL_RGBA8, dim, dim, array_size);
    glTextureParameteri(texobj, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTextureParameteri(texobj, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
}


void
texture_set::bind(int texunit)
{
    glBindTextureUnit(texunit, texobj);
}


void
texture_set::load(int slot, char const *filename)
{
    SDL_Surface* surf = IMG_Load( filename );

    if (!surf)
        errx(1, "Failed to load texture %d:%s", slot, filename);
    if (surf->w != dim || surf->h != dim)
        errx(1, "Texture %d:%s is the wrong size: %dx%d but expected %dx%d",
                slot, filename, surf->w, surf->h, dim, dim);

    /* just blindly upload as if it's RGBA/UNSIGNED_BYTE. TODO: support weirder things */
    glTextureSubImage3D(texobj, 0,
                    0, 0, slot,
                    dim, dim, 1,
                    surf->format->BytesPerPixel == 4 ? GL_RGBA : GL_RGB,
                    GL_UNSIGNED_BYTE,
                    surf->pixels);

    SDL_FreeSurface(surf);
}

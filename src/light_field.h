#pragma once


struct light_field {
    GLuint texobj;
    unsigned char data[128*128*128];

    light_field() : texobj(0) {
        glCreateTextures(GL_TEXTURE_3D, 1, &texobj);
        glTextureStorage3D(texobj, 1, GL_R8, 128, 128, 128);
    }

    void bind(int texunit)
    {
        glActiveTexture(GL_TEXTURE0 + texunit);
        glBindTexture(GL_TEXTURE_3D, texobj);
    }

    void upload()
    {
        /* TODO: toroidal addressing + partial updates, to make these uploads VERY small */
        /* TODO: experiment with buffer texture rather than 3D, so we can have the light field
         * persistently mapped in our address space */

        glTextureSubImage3D(texobj, 0, 0, 0, 0,
                        128, 128, 128,
                        GL_RED,
                        GL_UNSIGNED_BYTE,
                        data);
    }
};

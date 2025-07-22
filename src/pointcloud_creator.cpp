// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

namespace lang_sam_to_map{
    struct Point3D
    {
        float x;
        float y;
        float z;

        Point3D(){x = 0., y = 0., z = 0;}
    };

    struct RGB
    {
        int r;
        int g;
        int b;
        RGB(){r = 0., g = 0., b = 0;}
    };
    
    struct Point3DRGB
    {
        Point3D p;
        RGB c;
    };
    
    
}
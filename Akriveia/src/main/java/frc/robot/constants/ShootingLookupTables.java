package frc.robot.constants;

public class ShootingLookupTables{

    // First index of array elements corresponds to horizontal distance to Target (ie, deltaX as reported by Vision)
    // Second index of array elements corresponds to vertical distance to Target (ie, deltaY as reported by Vision)

    public static final double[][] RPM = {
        //           
        // horz dist         0ft, 4ft, 6ft, 8ft, 10ft, 12ft, 14ft, 16ft, 18ft
        /* vert dist  0ft */ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        /* vert dist  2ft */ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        /* vert dist  4ft */ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        /* vert dist  6ft */ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        /* vert dist  8ft */ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        /* vert dist 10ft */ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        /* vert dist 12ft */ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    };

    public static final double[][] HOODPOS_for2PosHood = {
        //           
        // horz dist         0ft, 4ft, 6ft, 8ft, 10ft, 12ft, 14ft, 16ft, 18ft
        /* vert dist  0ft */ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        /* vert dist  2ft */ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        /* vert dist  4ft */ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        /* vert dist  6ft */ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        /* vert dist  8ft */ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        /* vert dist 10ft */ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        /* vert dist 12ft */ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    };

    public static final double[][] DRUMVEL= {
        //           
        // horz dist         0ft, 4ft, 6ft, 8ft, 10ft, 12ft, 14ft, 16ft, 18ft
        /* vert dist  0ft */ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        /* vert dist  2ft */ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        /* vert dist  4ft */ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        /* vert dist  6ft */ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        /* vert dist  8ft */ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        /* vert dist 10ft */ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        /* vert dist 12ft */ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    };
}

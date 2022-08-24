socket_height=14;
socket_width=33.7;
box_width=30;
box_height=70;
box_depth=20;
wall_thickness=1.8;

difference() {
    cube([box_width,box_depth,box_height],center=true);
    
    translate([0,0,wall_thickness]) {
        cube([box_width-wall_thickness*2,box_depth-wall_thickness*2,box_height],center=true);
    }
    translate([4,box_depth/2,box_height/2-socket_width/2]) {
        cube([socket_height,10,socket_width+0.2],center=true);
    }    
}

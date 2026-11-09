$fn=50;

difference(){
    union(){
hull(){
    translate([-1,2,0])cylinder(d=4,h=5);
    translate([54-2,2,0])cylinder(d=4,h=5);
    translate([54-2,32-2,0])cylinder(d=4,h=5);
    translate([-1,32-2,0])cylinder(d=4,h=5);
}
translate([10,3,3])cube([30,26,17]);

translate([15,0,5])hull(){
    cube([2,3,0.5]);
    translate([0,3,13])cube([2,1,0.5]);
}
translate([33,0,5])hull(){
    cube([2,3,0.5]);
    translate([0,3,13])cube([2,1,0.5]);
}
translate([33,29,5])hull(){
    cube([2,3,0.5]);
    translate([0,-1,13])cube([2,1,0.5]);
}
translate([15,29,5])hull(){
    cube([2,3,0.5]);
    translate([0,-1,13])cube([2,1,0.5]);
}

}

translate([-0.5,5.25,2])cube([51,21.5,17]);

translate([9,6.25,3])cube([32,19.5,26]);

translate([5,2.5,0])cylinder(d=3,h=10);
translate([50-5,2.5,0])cylinder(d=3,h=10);
translate([50-5,32-2.5,0])cylinder(d=3,h=10);
translate([5,32-2.5,0])cylinder(d=3,h=10);

translate([5,2.5,2])cylinder(d=6,h=10);
translate([50-5,2.5,2])cylinder(d=6,h=10);
translate([50-5,32-2.5,2])cylinder(d=6,h=10);
translate([5,32-2.5,2])cylinder(d=6,h=10);

translate([-4,11,2])cube([10,10,10,]);
}
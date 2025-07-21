// Base dimensions (adjust as needed)
board_width = 21;
board_length = 17.5;
board_height = 3.5;

// Enclosure dimensions
enclosure_width = board_width + 5;
enclosure_length = board_length + 5;
enclosure_height = board_height + 5;

// Cutout for USB-C
usb_cutout_width = 10;
usb_cutout_height = 5;

module base() {
  translate([0, 0, 0]) cube([enclosure_width, enclosure_length, enclosure_height]);
}

module usb_cutout() {
  translate([enclosure_width/2 - usb_cutout_width/2, enclosure_length - 2, enclosure_height/2 - usb_cutout_height/2]) cube([usb_cutout_width, 3, usb_cutout_height]);
}

difference() {
    base();
    usb_cutout();
}
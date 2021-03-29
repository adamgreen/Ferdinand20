/*  Copyright (C) 2021  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
// Computer vision experiments to detect traffic cones.
use opencv::{core, highgui, prelude::*, videoio};

fn main() {
    run().unwrap()
}

fn run() -> opencv::Result<()> {
    // Red-only pixels with values less than this value are considered not to be red.
    // let red_threshold = 64;

    // The color channels in the OpenCV RGB frame.
    let blue_channel = 0;
    let green_channel = 1;
    let red_channel = 2;


    let video_window = "Video";
    highgui::named_window(video_window, 1)?;
    let red_window = "Red Only";
    highgui::named_window(red_window, 1)?;
    let mut cam = videoio::VideoCapture::new(0, videoio::CAP_ANY)?; // 0 is the default camera
    let opened = videoio::VideoCapture::is_opened(&cam)?;
    if !opened {
        panic!("Unable to open default camera!");
    }
    loop {
        let mut frame = core::Mat::default()?;
        cam.read(&mut frame)?;
        let frame_size = frame.size()?;
        if frame_size.width > 0 {
            highgui::imshow(&video_window, &mut frame)?;

            let row_count = frame_size.height as usize;
            let col_count = frame_size.width as usize;
            let mut new_image: Vec<Vec<u8>> = Vec::<Vec<u8>>::with_capacity(row_count);
            let frame = frame.to_vec_2d::<core::Vec3b>()?;
            for row in frame.iter() {
                let mut new_row = Vec::<u8>::with_capacity(col_count);
                for col in row.iter() {
                    let red = col[red_channel] as i32;
                    let green = col[green_channel] as i32;
                    let blue = col[blue_channel] as i32;
                    let red_value = match red-green-blue {
                        x if x < 0 => 0u8,
                        x => x as u8
                    };
                    new_row.push(red_value as u8);
                }
                new_image.push(new_row);
            }
            let mut new_image = Mat::from_slice_2d(&new_image)?;
            highgui::imshow(red_window, &mut new_image)?;
        }

        let key = highgui::wait_key(8)?;
        if key > 0 && key != 255 {
            break;
        }
    }

    Ok(())
}

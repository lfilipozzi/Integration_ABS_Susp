function createVideoHalfCar(frame,video_filename,fps)

% Create a temporary working folder to store the image sequence.
workingDir = tempname;
mkdir(workingDir)
mkdir(workingDir,'images')

% Loop through the video, reading each frame into a width-by-height-by-3 
% array named img. Write out each image to a JPEG file with a name in the 
% form imgN.jpg, where N is the frame number.
for ii=1:numel(frame)
   image_filename = [sprintf('%03d',ii) '.jpg'];
   image_fullname = fullfile(workingDir,'images',image_filename);
   imwrite(frame(ii).cdata,image_fullname) % Write out to a JPEG file  
                                           % (img1.jpg, img2.jpg, etc.)
end

% Find all the JPEG file names in the images folder. Convert the set of 
% image names to a cell array.
imageNames = dir(fullfile(workingDir,'images','*.jpg'));
imageNames = {imageNames.name}';

% Construct a VideoWriter object, which creates a Motion-JPEG AVI file by 
% default.
video_fullname = fullfile(workingDir,video_filename);
outputVideo = VideoWriter(video_fullname);
outputVideo.FrameRate = fps;
open(outputVideo)

% Loop through the image sequence, load each image, and then write it to 
% the video.
for ii = 1:length(imageNames)
   img = imread(fullfile(workingDir,'images',imageNames{ii}));
   writeVideo(outputVideo,img)
end

close(outputVideo)

% Move the video file to the folder video
movefile(video_fullname,video_filename);

end




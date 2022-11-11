%% Denoise GUI
%{
This script runs a graphical user interface to go through different
smoothing/denoising parameters of an image, series of images, or a video.
It can take as input:

filename --> as a path to an image or video file
            e.g filename = '/Users/username/folder/file.avi';
            e.g filename = '/Users/username/folder/file.tif';
         --> OR as a path to a folder of images that, when strung together
         create a movie. 
            e.g filename = '/Users/username/folder/';
             NOTE: The order that they are read in, depends on the
             name of the file, so [image1.tif image2.tif image3.tif etc.] will
             create a video in that order, however, if the filenames are not 
             alphabetically in the right temporal order, matlab will not read
             them in in the correct sequence!

It can export an MP4 movie with the desired tuning parameters and will put
the file in the same directory as specified with the tag:
filename_dn_#_#.mp4

-------------------
GUI parameters:

Frame Number [1 #frames]
Noise Threshold Factor [0 1] --> This takes the maximum luminance value
    from each color channel and assigns a threshold for the noise as a
    percentage of the max lumanince value of the image. e.g. a value of .5
    would consider a value that is half the brightest luminance and under
    pixels with noise.
Attenuation Factor [0 1] --> This is the level of attenuation applied to
    the pixels deemed noise. The closer it is to 1 the more it maintains its
    luminace value from the original
Gaussian Smoothing Factor [1 200] --> This is the radius to smooth the
    mask created from the previous two parameters. A value of one is no
    smoothing and higher values increase the radius of smoothing, allowing for
    even falloff of the attenuation. (basically making it so there are no hard
    lines from the effect.

Play Button --> will play the video using the parameters specified
Export Button --> will export the video using the parameters specified

%}

filename = '/Users/danielcalbick/Desktop/DenethiDN/mSc_ATXN2_Q22_axon.avi';

run_dn_gui(filename);

%% Main Function

function run_dn_gui(filename)
%% Load Data

disp('Loading Data From File...')
userData = loadData(filename);

%% Create Figures
warning('off')

fig = figure('Position',[110 89 856 1026]); 
ax    = gobjects(3,1);
ttl   = {'Original' , 'Mask' , 'Denoised'};
for i = 1:3, ax(i) = subplot(3,1,i); title(ttl(i)), end

uifig = uifigure('Position' , [1000 450 550 550] , 'AutoResizeChildren' , 'off');

set(uifig,'SizeChangedFcn',@(src , evt) reSizeFunc(src , evt ));

userData.ttl      = ttl;

uifig.UserData  = userData;

set([fig uifig] , 'Tag' , 'dnFigs','DeleteFcn',@deleteFunc)

%% --------- Setup Functions --------------

nframes = userData.nframes;

nrows  = 5;
h      = uifig.Position(4);
r      = h/(nrows+1);

lblpos = @(x) [200 x.Position(2)+25 450 20];
sldpos = @(x) [ 50 x*r 450  3];

sldfunc = @(src , evt) sliderFunc(src , evt, ax , uifig);

if userData.nframes >1
frame_slider = uislider(uifig,...
    'Position',sldpos(5) ,...
    'Value' , 1 , ... 
    'Limits' , [1 nframes+.9],...
    'MajorTicks', 1:nframes , ...
    'Tag', 'frame', ...
    'ValueChangingFcn', sldfunc);
else,frame_slider = uislider(uifig,...
    'Position',sldpos(5) ,...
    'Value' , 1 , ... 
    'Limits' , [0 1],...
    'Tag', 'frame', ...
    'Enable','off');
end
uilabel(uifig,'Position',lblpos(frame_slider) ,...
    'Text' , 'Frame Number', 'FontSize', 14);

noiseThresh_slider = uislider(uifig,...
    'Position',sldpos(4) ,...
    'Value' , 0.4 , ... 
    'Limits' , [0 1],...
    'Tag', 'noise', ...
    'ValueChangingFcn', sldfunc);

uilabel(uifig,'Position',lblpos(noiseThresh_slider) ,...
    'Text' , 'Noise Threshold Factor', 'FontSize', 14 );

attenuation_slider = uislider(uifig,...
    'Position',sldpos(3) ,...
    'Value' , 0.2 , ... 
    'Limits' , [0 1],...
    'Tag', 'attenuation', ...
    'ValueChangingFcn', sldfunc);

uilabel(uifig,'Position', lblpos(attenuation_slider) ,...
    'Text' , 'Attenuation Factor', 'FontSize', 14 );

[h , w] = size(userData.frames,[1 2]);
lim =  min([h w])*0.04; lim = ceil(lim);
gauss_slider = uislider(uifig,...
    'Position',sldpos(2) ,...
    'Value' , lim/2 , ... 
    'Limits' , [1 lim],...
    'Tag', 'gauss', ...
    'ValueChangingFcn', sldfunc);

uilabel(uifig,'Position', lblpos(gauss_slider) ,...
    'Text' , 'Gaussian Smoothing Factor', 'FontSize', 14 );

btn = uibutton(uifig , 'state', ...
    'Position',[150 r 100 30] , 'Text' , 'Play',...
    'ValueChangedFcn', @(self , evt) buttonPush(self , evt , frame_slider, uifig , ax));

uibutton(uifig , 'push', ...
    'Position',btn.Position + [110 0 0 0] , 'Text' , 'Export',...
    'ButtonPushedFcn', @(self , evt) exportBtnFcn(self , evt , uifig));

%% --------- Initialize --------------
loadResizeData(uifig)

tmp.Tag = '';
sliderFunc(tmp , [] , ax , uifig)

warning('on')


end

%% Support Functions

function userData = loadData(filename)
   
    [pathstr, name, ext] = fileparts(filename);   

    if isfolder(filename)
        [frames , mov] = loadImageFile(filename); 
        fnameout = [ pathstr '/' name '/' name '_dn_'];
    else
        try imfinfo(filename); [frames , mov] = loadImageFile(filename); 
        catch, [frames , mov] = loadVideoFile(filename); end
        fnameout = [ pathstr '/' name '_dn_'];
    end
   
    userData.frames   = frames;
    userData.movin    = mov;
    userData.nframes  = size(frames,4);
    userData.fnameout = fnameout;

end

function [frames, mov] = loadImageFile(imagePath)


    imds = imageDatastore(imagePath);
    imgs = readall(imds);

    [h,w] = size(imgs,[1 2]);

    frame_size = [450 1620]; % increase and standardize frame size

    nframes = numel(imgs);

    frames = zeros([frame_size 3 nframes]);

    for i = 1:nframes
        im    = imgs{i};
        frame = imresize(im , frame_size);  
        frames(:,:,:,i) = frame;
    end


    mov.FrameRate = ceil(nframes/5);


end

function [frames, mov] = loadVideoFile(filename)

    mov        = VideoReader(filename);

    frame_size = [450 1620]; % increase and standardize frame size

    nframes = mov.NumFrames;
    frames  = uint8(zeros([frame_size  3 nframes]));

    count = 1;
    while hasFrame(mov)
    
        % Read in current frame
        frame = readFrame(mov);
        frame = imresize(frame,frame_size); % resize the video

        frames(:,:,:,count) = frame;
        count = count+1;
    
    end

end

function exportBtnFcn(~ , ~ , uifig)

    d       = uifig.UserData;
    nframes = size(d.frames,4);

    frm_sld = findobj(uifig , 'Tag'  , 'frame'); frameNum = frm_sld.Value;
    thr_sld = findobj(uifig , 'Tag'  , 'noise'); dnthresh = thr_sld.Value;
    atn_sld = findobj(uifig , 'Tag'  , 'attenuation'); mskNoise = atn_sld.Value;
    gau_sld = findobj(uifig , 'Tag'  , 'gauss'); gaussFac = gau_sld.Value;

    fnameout = [d.fnameout ...
        sprintf('%.0d_' , dnthresh) sprintf('%.0d_' , mskNoise)];

    hndls = uifig.Children;
    set(hndls,'Visible', false)
    str = ['Exporting File as:' newline fnameout ];
    lbl = uilabel(uifig,'Text' , str , 'FontSize', 30,...
        'Position', [50 200 500 200],'WordWrap','on');

    movout  = VideoWriter(fnameout, 'MPEG-4' );    
    movout.FrameRate  = d.movin.FrameRate;
    
    open(movout)% open video file


    for i = 1:nframes
        frame = d.frames(:,:,:,i);
           
        % Take the maximum value from each color channel 
        % noise probably is diffuse through all color channels so it should be
        % weaker than the actual objects
        mxFrame = double(max(frame,[],3)); 
        mask    = mxFrame/max(mxFrame(:)); % make that a percentage of the max total value
        
        % Threshold the noise via dnthresh so all values under it are pushed
        % darker
        idx         = mask < dnthresh; 
        mask(idx)   = mskNoise;
        mask(~idx)  = 1;
        
        % Smooth the transition between attenuated area and areas we are keeping
        mask      = imgaussfilt(mask,gaussFac);
        
        % Apply the mask to the frame
        imout = mask .* double(frame);
    
        % Write frame to output video file
        writeVideo(movout,uint8(imout))
    end
    delete(lbl)
    set(hndls,'Visible', true)

end

function buttonPush(self , ~ , frame_slider, uifig , ax)

nframes = size(uifig.UserData.frames,4);

i = 1;
while self.Value && i <= nframes
        self.Text = 'Pause';
        tmp.Tag = '';

        frame_slider.Value = i;
        sliderFunc(tmp , [] , ax , uifig)
        drawnow

        i = i+1;     
end

self.Text  = 'Play';
self.Value = false;

end

function sliderFunc(src , evt , ax , uifig)

frm_sld = findobj(uifig , 'Tag'  , 'frame'); frameNum = frm_sld.Value;
thr_sld = findobj(uifig , 'Tag'  , 'noise'); dnthresh = thr_sld.Value;
atn_sld = findobj(uifig , 'Tag'  , 'attenuation'); mskNoise = atn_sld.Value;
gau_sld = findobj(uifig , 'Tag'  , 'gauss'); gaussFac = gau_sld.Value;

switch src.Tag
    case 'frame', frameNum = evt.Value;
    case 'noise', dnthresh = evt.Value;
    case 'attenuation', mskNoise = evt.Value;
    case 'gauss', gaussFac = evt.Value;
end

frame = uifig.UserData.frames(:,:,:,floor(frameNum));
ttl   = uifig.UserData.ttl;

% Take the maximum value from each color channel 
% noise probably is diffuse through all color channels so it should be
% weaker than the actual objects
mxFrame = double(max(frame,[],3)); 
mask    = mxFrame/max(mxFrame(:)); % make that a percentage of the max total value

% Threshold the noise via dnthresh so all values under it are pushed
% darker
idx         = mask < dnthresh; 
mask(idx)   = mskNoise;
mask(~idx)  = 1;

% Smooth the transition between attenuated area and areas we are keeping
mask      = imgaussfilt(mask,gaussFac);

% Apply the mask to the frame
imout = mask .* double(frame);

% Plot
imshow( uint8(frame) , 'Parent',ax(1) ), title(ax(1) , ttl(1))
imagesc(max(mask,[],3)  , 'Parent',ax(2)), title(ax(2) , ttl(2))
ax(2).XAxis.Visible = 0;
ax(2).YAxis.Visible = 0;
imshow(uint8(imout) , 'Parent',ax(3) ), title(ax(3) , ttl(3))


end

function reSizeFunc(src , ~)

nw = src.Position(3);
nh = src.Position(4);

warning('off','all')
h = findall(src , '-property' , 'Position');
for i = 2:numel(h)
    
    if strcmp(h(i).Type , 'legend'), continue,end
    try ratio = h(i).UserData.ratio; catch, continue,end
    try h(i).Position = [nw nh nw nh].*ratio;catch,continue,end
    
end

t = findall(src , '-property' , 'FontSize');
for i = 1:numel(t)
    try fontratio = t(i).UserData.fontratio; catch, continue,end
    try t(i).FontSize = nw.*fontratio;catch,continue,end
    
end

warning('on','all')
end

function loadResizeData(f)

width  = f.Position(3);
height = f.Position(4);


h = findall(f , '-property' , 'Position');
for i = 2:numel(h)

    try ratio = h(i).Position./[width height width height]; 
    catch,h(i).UserData.ratio = [];continue,end
    h(i).UserData.ratio = ratio;
    
end

% t = findall(f , '-property' , 'FontSize');
% for i = 1:numel(t)
% 
%     try fontratio = (t(i).FontSize./(width/2)); 
%     catch,t(i).UserData.fontratio = [];continue,end
%     t(i).UserData.fontratio = fontratio;
%     
% end

end

function deleteFunc(~ , ~)
    delete(findall(0,'Tag' , 'dnFigs'))
end


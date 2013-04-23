

function [] = readFile()
hasDepth = 0;
depthWidth = 0;
depthHeight = 0;

hasRGB = 0;
RGBWidth = 0;
RGBHeight = 0;

file = fopen('C:\Users\ganterd\Desktop\Dissertation\KTMKinectRecorder\output\saved.bin');
fileInfo = char(fread(file,5)).'
hasDepth = strcmp(fileInfo, 'DEPTH');
if(hasDepth)
    fileInfo = char(fread(file,8)).'
    if(strcmp(fileInfo,'80X60   '))
        depthWidth = 80;
        depthHeight = 60;
    elseif(strcmp(fileInfo,'320X240 '))
        depthWidth = 320;
        depthHeight = 240;
    elseif(strcmp(fileInfo,'640X480 '))
        depthWidth = 640;
        depthHeight = 480;
    end
else
    fseek(file,-5,'cof')
end

    
fileInfo = char(fread(file,5)).'
hasRGB = strcmp(fileInfo, 'RGB  ');
if(hasRGB)
    fileInfo = char(fread(file,8)).'
    if(strcmp(fileInfo,'640X480 '))
        RGBWidth = 640;
        RGBHeight = 480;
    elseif(strcmp(fileInfo,'1290X960'))
        RGBWidth = 1290;
        RGBHeight = 960;
    end
else
    fseek(file,-5,'cof')
end


end
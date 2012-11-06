function [] = drawMap( )

    axis([1 500 1 1000])
    % Load in a background image and display it using the correct colors
    % The image used below, is in the Image Processing Toolbox. If you do not have %access to this toolbox, you can use another image file instead.
    I=imread('map-photo.png');
    imagesc(I)
    colormap gray
    hold on
    plot([100 200],[50,150],'g','linewidth',3)

end


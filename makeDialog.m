function h = makeDialog(sMsg)
% MAKEDIALOG M-file
% - Creates a modal (blocks all actions) dialog box given an input message
%   that is called when the GUI is busy with a process
% - Changes the cursor to a hourglass when the dialog box is created, and
%   back to an arrow when the dialog box is closed
% - The position of the dialog is set at the centered on the callback
%   figure if available, else, centered on the screen

% -------------------------------
% makeDialog.m
% - Written by Michael K.K. Leung
% - July 13, 2006
% -------------------------------
% To do...
% - ...

%******************** GUI Component UserData Summary **********************
% handles.sliderMain (slider)   For storing the previous value of slider

%**************************************************************************

sFigName = 'Status';
iWidth = 250;
iHeight = 75;

FigPos = get(0,'DefaultFigurePosition');
FigWidth = iWidth;
FigHeight = iHeight;

% Overload the command gcbf (get callback figure)
gcbf = findobj('Tag','figMain');
% -- Guide Start --
if isempty(gcbf)
    ScreenUnits=get(0,'Units');
    set(0,'Units','pixels');
    ScreenSize=get(0,'ScreenSize');
    set(0,'Units',ScreenUnits);

    FigPos(1)=1/2*(ScreenSize(3)-FigWidth);
    FigPos(2)=2/3*(ScreenSize(4)-FigHeight);
else
    GCBFOldUnits = get(gcbf,'Units');
    set(gcbf,'Units','pixels');
    GCBFPos = get(gcbf,'Position');
    set(gcbf,'Units',GCBFOldUnits);
    FigPos(1:2) = [(GCBFPos(1) + GCBFPos(3) / 2) - FigWidth / 2, ...
                   (GCBFPos(2) + GCBFPos(4) / 2) - FigHeight / 2];
end
FigPos(3:4)=[FigWidth FigHeight];
% -- Guide End --

% Change the figure pointer and disable slider
try
    handles = guidata(gcbf);
    set(handles.figMain,'Pointer','watch');
    curValue = get(handles.sliderMain,'Value');
    set(handles.sliderMain,'Enable','off','UserData',curValue);
end
% Make the dialog box
h = dialog('Position',FigPos,'Name',sFigName,'WindowStyle','normal','Units','pixels');
uicontrol(h,'Style','text','String',sMsg,'Units','pixels','FontSize',9,'Position',[18 27.5 210 20],'DeleteFcn',@onDeleteDiag);
drawnow;

function onDeleteDiag(src,eventdata)
hObject = findobj('Tag','figMain');
% Change back figure pointer and disable slider
try
    handles = guidata(findobj('Tag','figMain'));
    prevValue = get(handles.sliderMain,'UserData');
    set(handles.sliderMain,'Enable','on','Value',prevValue-1);
    % A fix that causes an update to sliderbar value (so matlab would move it)
    set(handles.sliderMain,'Value',prevValue);
    set(handles.figMain,'Pointer','arrow');
catch
    % disp('GUI window not found.');
end

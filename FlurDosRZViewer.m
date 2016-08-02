function varargout = FlurDosRZViewer(varargin)
% FLURDOSRZVIEWER M-file for FlurDosRZViewer.fig
% - A stand-alone GUI that displays 2D dose contours in different planes
% - Supports 3D doses from RTOG and DOSXYZnrc 3ddose files
% - Contains built-in tools for dose data analysis, and export to Excel

% -------------------------------
% FlurDosRZViewer.m
% - Written by Leighton Warmington
% - Based on work by Michael K.K. Leung, August 23, 2006
% -------------------------------
% To do...
% - 

%************************** GUIData Summary *******************************
% - s_Dose         Structure of dose from 3ddose/RTOG file
% - s_RefDose   Structure of RefDose dose from 3ddose/RTOG file
% - linepos        Position of slider, also of slice being viewed
% - iNumVoxels      Size of 3ddose phantom
% - X/Y/Z           Coordinates of 3ddose phantom
% - curLayer        Current layer being viewed ('X','Y','Z')
% - normDose        Dose used as maximum for normalization
% - RefDoseNormDose  Dose used as maximum for normalization for RefDose subtraction dose
% - x,y,zVox        Coordinate of the normalization point
% - doseType        Whether source dose is '3ddose' or 'RTOG'

%******************* Initialization Code (Do not edit) *******************
gui_Singleton = 0;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @FlurDosRZViewer_OpeningFcn, ...
                   'gui_OutputFcn',  @FlurDosRZViewer_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end

%**************************** Program Starts ******************************

% --- Executes just before FlurDosRZViewer is made visible.
function FlurDosRZViewer_OpeningFcn(hObject, eventdata, handles, varargin)

handles.GUIName = 'Flurz-Dosrz Viewer';
set(handles.figFlurzDosrzViewer,'Name',[handles.GUIName ' - Main Window']);

handles.Block=[];
handles.lineDose=[];
handles.lineError=[];
handles.pos=[];
handles.info=[];


% Update handles structure
guidata(hObject, handles);

% --- Outputs from this function are returned to the command line.
function varargout = FlurDosRZViewer_OutputFcn(hObject, eventdata, handles) 
try
    % Get default command line output from handles structure
    varargout{1} = handles.output;
end


                   % ------ FILE MENU ITEMS ------ %
% --------------------------------------------------------------------
function File_Open_Callback(hObject, eventdata, handles)
% Select the dose file
[sFilename,sFolder] = uigetfile({'*.plotdat','Plot Data'; '*.spectra','Spectra file'},'Browse for File');

if (sFilename==0)
    return;
end

cd(sFolder);
handles.path=sFolder;
 
% Read data file
h = makeDialog('Reading dose...');
% Check which type it is: 
extInd = max(find(sFilename=='.'));
sExt = sFilename(extInd+1:length(sFilename));
if (strcmpi(sExt,'plotdat'))
    [handles.dose, handles.plotinfo,handles.numBlocks] = readDoseFile([sFolder sFilename], sExt);
    handles.doseType = 'plotdat';
else
    [handles.dose, handles.plotinfo, handles.numBlocks] = readDoseFile([sFolder sFilename], sExt);
    handles.doseType = 'spectra';
end
try
    delete(h);
end


for ind=1:handles.numBlocks
     LboxNames(ind) = handles.plotinfo(ind) ;%compile cell array of names.
end

 set(handles.referenceplot,'string',LboxNames); 
 set(handles.plotlist,'Value',1);
 set(handles.plotlist,'string',''); 
 
 set(handles.plotlist,'string',LboxNames); 
 set(handles.plotlist,'max',length(LboxNames)); 


% Show the path 
fullPath = [sFolder sFilename];
set(handles.MainFileName,'String',fullPath);
     
handles=normalize(handles);
handles=plotslice_Callback(handles,handles.axisFluence);
% Update handles structure
guidata(hObject, handles);



% --- Executes on button press in useDmaxdose.
function useDmaxdose_Callback(hObject, eventdata, handles)
% hObject    handle to useDmaxdose (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of useDmaxdose
useDmaxdose=get(handles.useDmaxdose,'value');
Dmaxdosestr=(get(handles.MaxDoseGy,'string'));
Dmaxdose=str2double(Dmaxdosestr);
 if (isempty(Dmaxdosestr)||(Dmaxdose==0))     
        msgbox('Enter a none zero Dmax Dose')
        set(handles.useDmaxdose,'value',0);
        handles.dosescale=100;
      return ;
 end
 if useDmaxdose==1
     handles.dosescale=Dmaxdose;
 else
     handles.dosescale=100;
 end

plotslice_Callback(handles,handles.axisFluence);
guidata(hObject, handles);



% --- Executes on selection change in DmaxWhere.
function DmaxWhere_Callback(hObject, eventdata, handles)
% hObject    handle to DmaxWhere (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns DmaxWhere contents as cell array
%        contents{get(hObject,'Value')} returns selected item from DmaxWhere
handles=normalize(handles);
guidata(hObject, handles);
plotslice_Callback(handles,handles.axisFluence);



% --- Executes during object creation, after setting all properties.
function DmaxWhere_CreateFcn(hObject, eventdata, handles)
% hObject    handle to DmaxWhere (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
set(hObject, 'String', {'% Depth Dose', '% Depth Dose - Limited'});



function dmaxLimIndex_Callback(hObject, eventdata, handles)
% hObject    handle to dmaxLimIndex (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of dmaxLimIndex as text
%        str2double(get(hObject,'String')) returns contents of dmaxLimIndex as a double



% --- Executes during object creation, after setting all properties.
function dmaxLimIndex_CreateFcn(hObject, eventdata, handles)
% hObject    handle to dmaxLimIndex (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes on button press in plotslice
function handles=plotslice_Callback(handles,dispAxes)
% hObject    handle to plotslice (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

Errbar= get(handles.errorbars,'Value');
axes(dispAxes)
cla;

DmaxD=get(handles.useDmaxdose,'value');
useRef=get(handles.UseReference,'value');

list_entries = get(handles.plotlist,'String');
index_selected = get(handles.plotlist,'Value');

count=length(index_selected);

col = distinguishable_colors(count);
n=1;

while n < count+1   
legendlist = list_entries{index_selected(n)};
      
        leg{n}=legendlist;
        dose=handles.dose(index_selected(n));
        pos=dose{1}(:,1);  
        lineDose=dose{1}(:,2);   
        lineError=dose{1}(:,3).*lineDose; 
           
        if useRef==1        
            if DmaxD==0
             lineDose=lineDose/handles.normDose*100;
             lineError=dose{1}(:,2).*lineError/handles.normDose*100;
            else
             dscale=  str2double(get(handles.MaxDoseGy,'string'));
             lineDose=lineDose/handles.normDose*dscale;
             lineError=dose{1}(:,2).*lineError/handles.normDose*dscale;
            end   
        end
        
        h_lineDose(:,n)=lineDose;
        h_lineError(:,n)=lineError;
        h_pos(:,n)=pos;
                
        hold on  ;
        if Errbar==1                
            errorbar(pos,lineDose,lineError, 'Marker' , '.','Color',col(n,:));              
        else      
             plot(pos,lineDose,'Color',col(n,:))
        end
        
        if get(handles.printplotdata,'value')==1
           Slice= leg{n}
           SliceData=[pos,lineDose]
        end
        
        n=n+1;
end

handles.lineDose=h_lineDose;
handles.lineError=h_lineError;
handles.pos=h_pos;
handles.info=leg;

h=legend(leg);
axis tight
hold off

set(h,'Box','off');

   if useRef==0   
        if strcmp('spectra',handles.doseType)==1  
           ylabel('(Fluence/MeV)/incident fluence/MeV^-1');
        else
           ylabel('Fluence/incidence fluence');
        end
   else   
       if DmaxD==1
           ylabel('Dose (Gy)');          
       else
           ylabel('Dose (%)');     
       end
   end
   
   
   if strcmp('spectra',handles.doseType)==1
         xlabel('Energy (MeV)');                
    else
         xlabel('Position (cm)');    
   end
    

   
% --- Executes on button press in errorbars.
function errorbars_Callback(hObject, eventdata, handles)
% hObject    handle to errorbars (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of errorbars

handles=plotslice_Callback(handles,handles.axisFluence);
guidata(hObject, handles);


% --- Executes on button press in printplotdata.
function printplotdata_Callback(hObject, eventdata, handles)
% hObject    handle to printplotdata (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of printplotdata
handles=plotslice_Callback(handles,handles.axisFluence);
guidata(hObject, handles);



% --- Executes on button press in zoom.
function zoom_Callback(hObject, eventdata, handles)
% hObject    handle to zoom (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of zoom

if (get(handles.zoom,'Value')==1)
        try
        dragzoom(handles.figFlurzDosrzViewer,'on')    %works on versions below 2014b. Need complete rewrite for nre version
        end
else
    try
    dragzoom(handles.figFlurzDosrzViewer,'off')
    end
end


% --- Executes on button press in updateplots.
function updateplots_Callback(hObject, eventdata, handles)
% hObject    handle to updateplots (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles=plotslice_Callback(handles,handles.axisFluence);
guidata(hObject, handles);


  
% --- Executes on button press in UseReference.
function UseReference_Callback(hObject, eventdata, handles)
% hObject    handle to UseReference (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of UseReference
handles=plotslice_Callback(handles,handles.axisFluence);
guidata(hObject, handles);


% --- Executes on button press in normalize.
function normalize_Callback(hObject, eventdata, handles)
% hObject    handle to normalize (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles=normalize(handles);
handles=plotslice_Callback(handles,handles.axisFluence);
% Update handles structure
guidata(hObject, handles);



function handles=normalize(handles)
whichdmax=get(handles.DmaxWhere,'value');
dmaxlim=str2num(get(handles.dmaxLimIndex,'string'));
refplot=get(handles.referenceplot,'string');
refplotNum= get(handles.referenceplot,'Value');

handles.numBlocks;

%dose=handles.dose(1);
%handles.x=dose{1}(:,1);

RP=get(handles.referenceplot,{'String','Value'});
refplot=RP{1}{RP{2}};

switch refplot
    case refplot
        dose=handles.dose(refplotNum);
        handles.ref=dose{1}(:,2);   
end 

switch whichdmax
    case 1                   
           handles.normDose =  max(max(max(handles.ref)));                                                                      
    case 2   
           handles.normDose =  max(max(max(handles.ref(1:dmaxlim))));                                                                       
end


% --- Executes on button press in PDDshow.
function PDDshow_Callback(hObject, eventdata, handles)
% hObject    handle to PDDshow (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of PDDshow
plotslice_Callback(handles,handles.axisFluence);


% --- Executes on button press in screenshot.
function screenshot_Callback(hObject, eventdata, handles)
% hObject    handle to screenshot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

nameandpath=get(handles.MainFileName,'string');
[p, ~, ~] = fileparts(nameandpath) ;
cd(p);

I = getframe(handles.figFlurzDosrzViewer);

figure
imagesc(I.cdata);
set(gcf, 'Color', 'w');
axis off;


imwrite(I.cdata,'screenshot_X.png');
export_fig(gcf,'screenshot_X','-pdf','-p0.01');



function saveDoseLineSlice_Callback(hObject, eventdata, handles)
nameandpath=get(handles.MainFileName,'string');
[pathstr, filenameorig, ~] = fileparts(nameandpath) ;

ErrBar=get(handles.errorbars,'value');
if ErrBar==0
    PlotType='Dose';
else
    PlotType='Error';
end

figure('Name','Line Dose in Current GUI Display');
h = axes;
plotslice_Callback(handles,h);

set(gcf, 'Color', 'w');  
set(gcf,'PaperPositionMode','auto');

filename=[filenameorig '-'  PlotType '.fig'];
saveDataNamefig = fullfile(pathstr,filename);
saveas(gca,saveDataNamefig)    
filename=[filenameorig '-' PlotType '.pdf'];
saveDataNamefig = fullfile(pathstr,filename);
saveas(gca,saveDataNamefig);


% --- Executes on button press in ExpCurlineDose.
function ExpCurlineDose_Callback(hObject, eventdata, handles)
% hObject    handle to ExpCurlineDose (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles=plotslice_Callback(handles,handles.axisFluence);
pos=handles.pos;
lineDose=handles.lineDose;
lineError=handles.lineError;
info=handles.info;

expwhere=get(handles.exportwhere,'value');
warning('off','MATLAB:xlswrite:AddSheet');
switch expwhere
    case 1
      %  rows=length(lineDose);

        [sFileName,sSaveDir] = uiputfile('*.xlsx','Save Data As...');
        if(sFileName==0) 
            return
        else 
        sFileName=[sSaveDir sFileName];
           
        xlswrite(sFileName, pos,'Position', 'A2');
        xlswrite(sFileName, lineDose,'Line Dose', 'A2');
        xlswrite(sFileName, lineError,'Line Error', 'A2');
      
        xlswrite(sFileName, info,'Position', 'A1');
        xlswrite(sFileName, info,'Line Dose', 'A1');
        xlswrite(sFileName, info,'Line Error', 'A1');
        end
        
    case 2
        
        assignin('base','Position',pos);
        assignin('base','LineDose',lineDose);
        assignin('base','LineError',lineError);   
        assignin('base','Label',info);  
end


% --- Executes on selection change in exportfile.
function exportfile_Callback(hObject, eventdata, handles)
% hObject    handle to exportfile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns exportfile contents as cell array
%        contents{get(hObject,'Value')} returns selected item from exportfile
filetype=get(handles.exportfiletype,'value');
nameandpath=get(handles.MainFileName,'string');
[pathstr, filenameorig, ~] = fileparts(nameandpath) ;

switch filetype
    case 1
        %dose = handles.s_Dose.dose;
        dose = handles.idose;
        fname=  handles.path;
        VS=[str2num(get(handles.exportresol,'string'))]
        filename=[filenameorig '- Dicom'];
        dicomwritevolume(filename, dose, VS);
        
    case  2
        x=handles.xi;
        y=handles.yi;
        z=handles.zi;

        center=get(handles.roiCenter,'string');

        inumVox=handles.iNumVoxels;
        
         if get(handles.useNormref,'value')==0
        dmaxVox = [handles.xVox, handles.yVox, handles.zVox]; 
         else
        dmaxVox = [handles.xVoxref,handles.yVoxref, handles.zVoxref]; 
         end
       % position of normdose
        volume_dose = handles.idose; %(y,x,z)  
        normdose=handles.normDose;

        h = waitbar(0, 'Saving mat files...', 'WindowStyle', 'modal');
            waitbar(0,h);

        save('EGS_mat_dose.mat','volume_dose','x','y','z','normdose','dmaxVox','inumVox');
        close(h);
        
    case 3
        
             
        
end
    
    
    
% --- Executes during object creation, after setting all properties.
function exportfile_CreateFcn(hObject, eventdata, handles)
% hObject    handle to exportfile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function MaxDoseGy_Callback(hObject, eventdata, handles)
% hObject    handle to MaxDoseGy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of MaxDoseGy as text
%        str2double(get(hObject,'String')) returns contents of MaxDoseGy as a double


% --- Executes during object creation, after setting all properties.
function MaxDoseGy_CreateFcn(hObject, eventdata, handles)
% hObject    handle to MaxDoseGy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

             

% --- Executes on selection change in RefFoil.
function RefFoil_Callback(hObject, eventdata, handles)
% hObject    handle to RefFoil (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns RefFoil contents as cell array
%        contents{get(hObject,'Value')} returns selected item from RefFoil



% --- Executes during object creation, after setting all properties.
function RefFoil_CreateFcn(hObject, eventdata, handles)
% hObject    handle to RefFoil (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
set(hObject, 'String', {'Water','Upstream Foil','Downstream Foil'});
        


% --- Executes on selection change in referenceplot.
function referenceplot_Callback(hObject, eventdata, handles)
% hObject    handle to referenceplot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns referenceplot contents as cell array
%        contents{get(hObject,'Value')} returns selected item from referenceplot

handles=normalize(handles);
plotslice_Callback(handles,handles.axisFluence);
guidata(hObject, handles);



% --- Executes during object creation, after setting all properties.
function referenceplot_CreateFcn(hObject, eventdata, handles)
% hObject    handle to referenceplot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end




% --------------------------------------------------------------------
function about_Callback(hObject, eventdata, handles)
% hObject    handle to about (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

msgbox('FlurDOSnrc Viewer, Version 1.0, Leighton Warmington, University of Minnesota' )


% --- Executes on button press in GDerror.
function GDerror_Callback(hObject, eventdata, handles)
% hObject    handle to GDerror (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of GDerror
plotslice_Callback(handles,handles.axisFluence);


function GDerrorPercent_Callback(hObject, eventdata, handles)
% hObject    handle to GDerrorPercent (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of GDerrorPercent as text
%        str2double(get(hObject,'String')) returns contents of GDerrorPercent as a double
plotslice_Callback(handles,handles.axisFluence);

% --- Executes during object creation, after setting all properties.
function GDerrorPercent_CreateFcn(hObject, eventdata, handles)
% hObject    handle to GDerrorPercent (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function GDerrormm_Callback(hObject, eventdata, handles)
% hObject    handle to GDerrormm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of GDerrormm as text
%        str2double(get(hObject,'String')) returns contents of GDerrormm as a double


% --- Executes during object creation, after setting all properties.
function GDerrormm_CreateFcn(hObject, eventdata, handles)
% hObject    handle to GDerrormm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function doseCorrection_Callback(hObject, eventdata, handles)
% hObject    handle to doseCorrection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of doseCorrection as text
%        str2double(get(hObject,'String')) returns contents of doseCorrection as a double
handles.dosecorrection=str2num(get(handles.doseCorrection,'string'));

% --- Executes during object creation, after setting all properties.
function doseCorrection_CreateFcn(hObject, eventdata, handles)
% hObject    handle to doseCorrection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes on selection change in exportwhere.
function exportwhere_Callback(hObject, eventdata, handles)
% hObject    handle to exportwhere (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns exportwhere contents as cell array
%        contents{get(hObject,'Value')} returns selected item from exportwhere


% --- Executes during object creation, after setting all properties.
function exportwhere_CreateFcn(hObject, eventdata, handles)
% hObject    handle to exportwhere (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
set(hObject, 'String', {'Excel', 'Workspace'});



% --- Executes on mouse press over axes background.
function axisFluence_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to axisFluence (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on selection change in plotlist.
function plotlist_Callback(hObject, eventdata, handles)
% hObject    handle to plotlist (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns plotlist contents as cell array
%        contents{get(hObject,'Value')} returns selected item from plotlist


% --- Executes during object creation, after setting all properties.
function plotlist_CreateFcn(hObject, eventdata, handles)
% hObject    handle to plotlist (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes on key press with focus on figFlurzDosrzViewer or any of its controls.
function figFlurzDosrzViewer_WindowKeyPressFcn(hObject, eventdata, handles)
% hObject    handle to figFlurzDosrzViewer (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.FIGURE)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on key press with focus on figFlurzDosrzViewer or any of its controls.
function figFlurzDosrzViewer_WindowButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to figFlurzDosrzViewer (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.FIGURE)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on key press with focus on figFlurzDosrzViewer and none of its controls.
function figFlurzDosrzViewer_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to figFlurzDosrzViewer (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.FIGURE)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)


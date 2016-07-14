import c4d, os, subprocess, tempfile, array, logging, threading, datetime, time, math, shutil

PBRT_EXPORT_ID = 1032340

DLG_PBRT                        = 60000
IDS_PBRT                        = 60001
IDC_PBRT_START                  = 60002
IDC_PBRT_LOG                    = 60003
IDC_PBRT_EXE                    = 60004
IDC_PBRT_MODE                   = 60005
IDC_PBRT_MODE_EXPORT            = 60006
IDC_PBRT_MODE_EXPORT_AND_RENDER = 60007
IDC_PBRT_MODE_RENDER            = 60008
IDC_PBRT_SAMPLES                = 60009
IDC_PBRT_ABORT                  = 60010
IDC_BUTTON_GROUP                = 60011
IDC_PBRT_LOGLEVEL               = 60012
IDC_PBRT_LOGLEVEL_DEBUG         = 60013
IDC_PBRT_LOGLEVEL_INFO          = 60014
IDC_PBRT_LOGLEVEL_WARNING       = 60015
IDC_PBRT_LOGLEVEL_ERROR         = 60016
IDC_PBRT_INTENSITY              = 60017

#IDC_TEMP_BAKE_LINK             = 60011 # temporary location to store the link to the bake texture tag

IDS_PBRT_START                  = 60100
IDS_PBRT_ABORT                  = 60101

# treeview column ids
TREEVIEW_COLUMN_ID_TIME         = 123
TREEVIEW_COLUMN_ID_LEVEL        = 234
TREEVIEW_COLUMN_ID_DESCRIPTION  = 345

MSG_PBRT_UPDATE_LOG             = 1000002
MSG_PBRT_FINISHED               = 1000003
MSG_PBRT_BAKETEXTURE            = 1000004

logpipe = None
g_bmp = None
g_thread = None

# although we only need a reference to the bake tag we also save a global reference to the document
# to keep the tag alive - without this the document would be deleted when out of scope.
g_bakeDoc = None
g_bakeTag = None
g_bakeTextureFile = None # when a baked environment is created, store the resulting file here
g_nLightSourcesExported = 0 # to determine whether to use auto light or not

""" The ErrorHandler inserts informations about errors and warning into this lists which will be displayed in the treeview """
g_errorlist = [] # example element{'time': '00:04:01', 'level':'Info', 'msg':'Oh my god!'}
g_errorRoot = None

g_levelMap = { 'DEBUG':logging.DEBUG, 'INFO':logging.INFO, 'WARNING':logging.WARNING, 'ERROR':logging.ERROR}
g_level = logging.INFO # filter level selected in dialog

logger = logging.getLogger(__name__)


"""Wrapper class which can be used to write to a logfile during a subprocess.call

http://codereview.stackexchange.com/questions/6567/how-to-redirect-a-subprocesses-output-stdout-and-stderr-to-logging-module
"""
class LogPipe(threading.Thread):
    def __init__(self):
        """Setup the object with a logger and a loglevel
        and start the thread
        """
        threading.Thread.__init__(self)
        self.daemon = False
        self.fdRead, self.fdWrite = os.pipe()
        self.pipeReader = os.fdopen(self.fdRead)
        self.start()

    def fileno(self):
        """Return the write file descriptor of the pipe
        """
        return self.fdWrite

    def run(self):
        """Run the thread, logging everything.
        """
        for line in iter(self.pipeReader.readline, ''):
            line = line.rstrip('\r\n')
            if len(line) > 0:
                # check if this message comes from the publish subprocess (LEVEL: msg..)
                tokens = line.split(':')
                if tokens > 0 and g_levelMap.has_key(tokens[0]):
                    level = tokens[0]
                    msg = line[len(tokens[0])+1:].strip()
                    # log this message with the correct level
                    # we are using warning and error explicetely because log()
                    # does not support our counting of warnings and errors
                    if level == 'WARNING':
                        logger.warning(msg)
                    elif level == 'ERROR':
                        logger.error(msg)
                    else:
                        logger.log(g_levelMap[level], msg)
                else:
                    if self.handleAsError(line):
                        logging.error(line)
                    elif self.handleAsInfo(line):
                        logging.info(line)
                    # ignore memory leak outputs
                    elif not 'Bytes' in line:
                        logging.debug(line)

        self.pipeReader.close()

    def close(self):
        """Close the write end of the pipe.
        """
        try:
            os.close(self.fdWrite)
        except:
            pass

    def handleAsInfo(self, msg):
        if ('rendering') in msg.lower():
            return True
        return False

    def handleAsError(self, msg):
        """Specifies if the given message should be handled
        as an error.
        """
        if ('failed') in msg.lower():
            return True
        if ('unable') in msg.lower():
            return True
        if ('wrong') in msg.lower():
            return True
        return False

    def __del__(self):
        """delete object, call close
        """
        self.close()


class ErrorItem(object):
    def __init__(self, time="", level="", msg=""):
        super(ErrorItem, self).__init__()
        self.children = []
        self.parent = None

        self.time = time
        self.level = level
        self.msg = msg

        self.is_selected = False

    def GetChildren(self):
        return self.children

    def AddChild(self, child):
        if not isinstance(child, ErrorItem):
            raise TypeError('expected TreeViewItem instance.')
        self.children.append(child)
        child.parent = self

    def GetNext(self):
        if not self.parent:
            return None
        try:
            i = self.parent.children.index(self)
            return self.parent.children[i + 1]
        except (ValueError, IndexError):
            return None

    def GetPred(self):
        if not self.parent:
            return None
        i = self.parent.children.index(self)
        if i > 0:
            return self.parent.children[i - 1]
        return None

    def GetUp(self):
        return self.parent

    def GetDown(self):
        try:
            return self.children[0]
        except IndexError:
            return None

    def GetRoot(self):
        curr = self
        while curr.parent:
            curr = curr.parent
        return curr

    def DeselectAll(self):
        self.is_selected = False
        for child in self.children:
            child.DeselectAll()

    def GetSelected(self):
        if self.is_selected:
            yield self
        for child in self.children:
            for selected in child.GetSelected():
                yield selected

    def Select(self, selected=True):
        self.is_selected = selected

    def IsSelected(self):
        return self.is_selected

    def Format(self, i=0):
        print i * '    ' + '<ROOT>'
        for child in self.children:
            child.format(i + 1)

    def Sort(self, key=lambda x: int(x.data['id'])):
        for child in self.children:
            child.sort(key)
        self.children.sort(key=key)

    def Find(self, key):
        if key(self):
            return self
        for child in self.children:
            r = child.find(key)
            if r: return r
        return None

class RootItem(ErrorItem):

    def __init__(self):
        super(RootItem, self).__init__()

    def GetSelected(self):
        return False

    def GetSelectedChildren(self):
        for child in self.children:
            for selected in child.GetSelected():
                yield selected

    def __str__(self):
        return "root"

    def Clear(self):
        self.children = []

    def GetLastChild(self):
        try:
            return self.children[-1]
        except IndexError:
            print "Ahaaaaa fu"
            return None

class ErrorHandler(logging.Handler):

    def __init__(self):
        # run the regular Handler __init__
        logging.Handler.__init__(self)
        del g_errorlist[:]

    def emit(self, record):
        now = datetime.datetime.now()
        time = datetime.datetime.strftime(now, '%H:%M:%S') # for more options see http://docs.python.org/2/library/datetime.html#strftime-strptime-behavior
        g_errorlist.append(ErrorItem(time, record.levelname, record.msg))
        UpdateErrorList()
        UpdateTreeView()

class ErrorList(c4d.gui.TreeViewFunctions):

    def GetFirst(self, root, userdata):
        """Return the first element of the hierarchy"""
        return root.GetDown()

    def GetNext(self, root, userdata, obj):
        """Return the next element of the passed object"""
        return obj.GetNext()

    def GetPred(self, root, userdata, obj):
        """Return the previous element of the passed object"""
        return obj.GetPred()

    def GetDown(self, data, userdata, obj):
        return obj.GetDown()

    def GetName(self, data, userdata, obj):
        return str(obj)

    def IsSelected(self, data, userdata, issue):
        return issue.is_selected

    def Select(self, data, userdata, obj, mode):
        if mode == c4d.SELECTION_SUB:
            obj.Select(False)
        elif mode == c4d.SELECTION_NEW:
            obj.GetRoot().DeselectAll()
            obj.Select()
        elif mode == c4d.SELECTION_ADD:
            bj.GetRoot().DeselectAll() # remove this to support multi selection
            obj.Select()

    def DrawCell(self, root, userdata, obj, col, drawinfo, bgColor):
        "Draw the cell"
        if col == TREEVIEW_COLUMN_ID_TIME:
            text = obj.time
        elif col == TREEVIEW_COLUMN_ID_LEVEL:
            text = obj.level.title()
        else:
            text = obj.msg

        area = drawinfo["frame"]

        # multiply text color with red or yellow depending on warning level (LAUBWERKCINEMA-524)
        textColor = area.GetColorRGB(c4d.COLOR_TEXT)
        textColor = dict((k, v / 255.0) for k, v in textColor.items())
        if obj.level == 'WARNING':
            # yellow
            textColor['b'] =  textColor['b'] * 0.0
        elif obj.level == 'ERROR':
            # multiply with red (pure red is barely readable)
            textColor['r'] = 1.0#textColor['r'] * 0.7
            textColor['g'] = 0.0#textColor['g'] * 0.1
            textColor['b'] = 0.0#textColor['b'] * 0.1

        area.DrawSetTextCol(c4d.Vector(textColor['r'], textColor['g'], textColor['b']), bgColor)
        area.DrawText(text, drawinfo["xpos"], drawinfo["ypos"], flags = c4d.DRAWTEXT_VALIGN_MASK)

    def GetLineHeight(self, root, userdata, obj, col, area):
        return 16

    def GetHeaderColumnWidth(self, root, userdata, col, area):
        width = 100
        if col == TREEVIEW_COLUMN_ID_TIME:
            width = 40
        elif col == TREEVIEW_COLUMN_ID_LEVEL:
            width = 50
        elif col == TREEVIEW_COLUMN_ID_DESCRIPTION:
            width = -1
        return width

    def GetColumnWidth(self, root, userdata, obj, col, area):
        width = 100
        if col == TREEVIEW_COLUMN_ID_TIME:
            width = 50
        elif col == TREEVIEW_COLUMN_ID_LEVEL:
            width = 60
        elif col == TREEVIEW_COLUMN_ID_DESCRIPTION:
            def lenMsg(x):
                return len(str(x.msg))
            width = area.DrawGetTextWidth(max(g_errorRoot.GetChildren(), key=lenMsg))
        return width

def UpdateErrorList():
    global g_errorRoot
    g_errorRoot.Clear()
    for error in g_errorlist:
        if g_levelMap[error.level] >= g_level:
            g_errorRoot.AddChild(error)

def UpdateTreeView():
    c4d.SpecialEventAdd(MSG_PBRT_UPDATE_LOG)

#
# helper functions to iterate through the object tree starting at a certain object
#
def iter_tree(obj, include_first=True):
    if include_first:
        yield obj
    cache = obj.GetCache()
    while cache:
        for o in iter_tree(cache, True):
            yield o
        cache = cache.GetNext()

    for child in obj.GetChildren():
        for o in iter_tree(child, True):
            yield o

#
# helper functions to iterate through all object trees in a document
#
def iter_all(doc):
    for obj in doc.GetObjects():
        for o in iter_tree(obj):
            yield o

#
# takes a name and an optional list of existing names
# the name is first forced to conform to a number of conditions (ascii only, etc.), then checked if
# it already exists and modified until it is unique
#
def SanitizeObjectName(name, nameList=None):
    # todo
    return name

def FindNamedObject(object, namedObjects):
    for namedObject in namedObjects:
        if object == namedObject[0]:
            return namedObject
    return None

#
# The GenerateTexturePath() function only exists in the C++ API, so we have to roll our own here...
#
def GenerateTexturePath(docpath, file):
    if os.path.isabs(file):
        if os.path.isfile(file):
            return file
        else:
            return ''

    if os.path.isfile(os.path.join(docpath, file)):
        return os.path.join(docpath, file)

    if os.path.isfile(os.path.join(docpath, 'tex', file)):
        return os.path.join(docpath, 'tex', file)

    # TODO: include texture paths from preferences (Edit->Preferences->Files einstellen)

    return ''


#
# This function walks up to the parents and collects all texture tags. It starts at the topmost parent
# object and grabs all tags, then goes back through the hierarchy and replaces all tags that are
# superceded.
# The order of the tags is inverted over the order on the objects, so the tags with the highest
# precedence come first. This means, as soon as a tag is found that will be applied to tthe piece of
# geometry we are looking at, we don't have to look any further
#
def GetTextureTagsRec(obj):
    parentTags = []
    parent = obj.GetUp()
    if parent:
        parentTags = GetTextureTagsRec(parent)
    else:
        parent = obj.GetCacheParent()
        if parent:
            parentTags = GetTextureTagsRec(parent)

    objectTags = [] # here we will collect all tags that will be added from the current object
    for tag in reversed(obj.GetTags()):
        # every texture tag that comes after one that has no restriction, can be ignored
        if tag.GetType() == c4d.Ttexture:
            # if there is no material associated, ignore this tag
            if not tag.GetMaterial():
                continue

            # if this texture tag has no restriction, it just overrides everything from parent objects
            # and also no other tag from the current object will be taken into account.
            restriction = tag[c4d.TEXTURETAG_RESTRICTION]
            if not restriction or len(restriction) == 0:
                objectTags.append(tag)
                parentTags = []
                break

            objectTags.append(tag)

            def hasRestriction(tag, restriction):
                if restriction == tag[c4d.TEXTURETAG_RESTRICTION]:
                    return True
                else:
                    return False

            # check if a tag with the identical restriction exists in parentTags and remove it
            parentTags = [parentTag for parentTag in parentTags if not hasRestriction(parentTag, restriction)]

    # insert the new tags at the beginning of the list
    objectTags.extend(parentTags)
    return objectTags

EPSILON = 0.00001

#
# depending on the mode, either just pass the texture name through or copy it to the target directory and return just the basename.
# add the texture to the textureList, so it can be cleared later
#
def ManageTexture(filename, doc, directory = None, textureList = None):
    filePath = GenerateTexturePath(doc.GetDocumentPath(), filename)
    if not os.path.isfile(filePath):
        logger.warning(filename + ' not found!')
        return filename

    if directory:
        # make sure dir really only contains the folder
        if not os.path.isdir(directory):
            raise ValueError(str(directory) + ' is not a directory!')

        # copy texture file to target path
        myFilename = os.path.basename(filePath)
        myDstPath = os.path.join(directory, myFilename)
        shutil.copyfile(filePath, myDstPath)

        # add (absolute!) destination path to texture list
        if textureList:
            textureList.append(myDstPath)

        # return just the base file name
        return myFilename
    else:
        return filePath


def makePbrtAttributeFloat(attrib, val):
    return '"float ' + attrib +'" [' + str(val) + ']'

def makePbrtAttributeColor(attrib, color, transform=True):
    color = c4d.utils.TransformColor(color, c4d.COLORSPACETRANSFORMATION_SRGB_TO_LINEAR) if transform else color
    return '"rgb ' + attrib + '" [' + str(color.x) + ' ' + str(color.y) + ' ' + str(color.z) +']'

def makePbrtAttributeTexture(attrib, tex):
    return '"texture ' + attrib + '" "' + tex + '"'

def makePbrtAttribute(attrib, val, transform=True):
    attribType = type(val)
    if attribType == float:
        return makePbrtAttributeFloat(attrib, val)
    elif attribType == c4d.Vector:
        return makePbrtAttributeColor(attrib, val, transform)
    elif attribType == str:
        return makePbrtAttributeTexture(attrib, val)

def makePbrtTexture(filename, texAbbrev, texType='spectrum', useGamma=True):
    extension = os.path.splitext(filename)[1]
    gamma = 1.0 if extension != '.exr' else 2.2
    if not (extension == '.exr' or extension == '.tga' or extension == '.pfm' or extension == '.png'):
        logger.warning('Texture file "' + os.path.basename(filename) + '" uses unsupported texture file format')
    bumpTextureName = os.path.splitext(os.path.basename(filename))[0] + '_' + texAbbrev
    bumTextureString = 'Texture "' + bumpTextureName + '" "' + texType + '" "imagemap" "string filename" "' + filename.replace('\\','\\\\') + '"'
    if not c4d.utils.CompareFloatTolerant(gamma, 1.0):
        bumTextureString += ' ' + makePbrtAttributeFloat('gamma', gamma)
    return bumpTextureName, bumTextureString

#
# export the given object as a polygon object
# since in pbrt an object can only have a single object, this may actually export as
# several pbrt objects
#
def ExportPolygonObject(pbrtGeometry, pbrtMaterials, exportedMaterials, obj, indent=""):
    if obj.GetType() != c4d.Opolygon:
        raise TypeError("Expected a BaseObject of type Opolygon!")

    doc = obj.GetDocument()
    pbrtDir = os.path.dirname(os.path.abspath(pbrtGeometry.name))
    
    #
    # TODO:
    # to split up, do the following
    # walk through polygons, for each vertex check if there is already a vertex that has:
    #   (1) the same vertex index
    #   (2) the same normal (within a limit)
    #   (3) the same uv coordinate (within a limit)
    # this process will create a map that for each vertex has a list of taget vertices with different uvs and normals
    # To accomodate for different primitives, we may either want to do this for each
    # material selection (potentially killing runtime behavior) or update the mapping with
    # new target vertex indices in for each material (probably more sensible).
    # when writing the polygons
    # That data structure will contain some information that lets us look up the original
    # data for comparison. An efficient way is probably to not copy the original data, but
    # point to the polygon that this data is stored with.
    # The number of different material assignements should be determined beforehand. Take
    # into consideration that this might be nontrivial to figure out based on material
    # sidedness, selections and material application order. Since we're making a 'grungy'
    # exporter, we should probably ignore a few of the intricacies for now.
    #
    # Also keep in mind that PBRT likes to get the texture passed directly to the mesh for
    # clipping via the alpha texture parameter.
    #
    
    # grab phong normals and uvw tag. The uvw tag can be different depending on which material we are writing
    tmpPhongNormals = obj.CreatePhongNormals()
    uvwTag = obj.GetTag(c4d.Tuvw)

    # keep a list of polygons already exported with a particular material, so we can stop when all polygons have been written
    # and also make sure all polygons get written even if there is no material assigned
    touchedPolygons = obj.GetPolygonS().GetClone()
    touchedPolygons.DeselectAll()

    # for all potentially applied materials, check if there are any selections and export a trianglemesh per selection 
    # we append an empty texturetag entry at the end, so there is an iteration that would write all polygons with the default material 
    textureTags = GetTextureTagsRec(obj)
    textureTags.append(c4d.BaseTag(c4d.Ttexture))
    for textureTag in textureTags:

        # create a list of just polygon selection tags
        def isPolygonSelectionTag(tag):
            if tag.GetType() == c4d.Tpolygonselection:
                return True
            else:
                return False

        selectionTags = [tag for tag in obj.GetTags() if isPolygonSelectionTag(tag)]
        restriction = textureTag[c4d.TEXTURETAG_RESTRICTION]
        currentSelection = None
        for selectionTag in selectionTags:
            if selectionTag.GetName() == restriction:
                currentSelection = selectionTag.GetBaseSelect()
                break

        if not currentSelection and restriction and len(restriction) > 0:
            #print "ignored due to no matching selection"
            continue

        if currentSelection and currentSelection.GetCount() == 0:
            #print "ignored due to empty selection"
            continue

        innerAttributeBlock = False
        alphaTextureName = None
        material = textureTag.GetMaterial()
        if material:
            # DEBUG
            #print material.GetName()

#            if material.GetType() != c4d.Mmaterial and material.GetType() != c4d.BaseMaterial:
#                logger.warning(material.GetName())
#                logger.warning("Unidentified material type encountered and ignored!")
#                logger.warning(material)
#                continue
        
            pbrtGeometry.write(indent + 'AttributeBegin\n')
            indent += "\t"
            pbrtGeometry.write(indent + 'NamedMaterial "' + str(material.GetName()) + '"\n')
            innerAttributeBlock = True

            # translate material
            if not material.GetName() in exportedMaterials:
                exportedMaterials[material.GetName()] = 1
                baseColor = c4d.Vector(0, 0, 0)
                colorTextureName = None
                if material[c4d.MATERIAL_USE_COLOR]:
                    baseColor = material[c4d.MATERIAL_COLOR_COLOR] * material[c4d.MATERIAL_COLOR_BRIGHTNESS]
                    colorShader = material[c4d.MATERIAL_COLOR_SHADER]
                    if colorShader and colorShader.GetType() == c4d.Xbitmap:
                        try:
                            colorTextureFile = ManageTexture(colorShader[c4d.BITMAPSHADER_FILENAME], doc, pbrtDir, None)
                            colorTextureName, colorTextureString = makePbrtTexture(colorTextureFile, 'color')
                        except (RuntimeError, ValueError) as err:
                            logger.error(err)
                        else:
                            pbrtMaterials.write(colorTextureString + '\n')
            
                alphaTextureName = None
                if material[c4d.MATERIAL_USE_ALPHA]:
                    alphaShader = material[c4d.MATERIAL_ALPHA_SHADER]
                    if alphaShader and alphaShader.GetType() == c4d.Xbitmap:
                        try:
                            alphaTextureFile = ManageTexture(alphaShader[c4d.BITMAPSHADER_FILENAME], doc, pbrtDir, None)
                            # alpha is stored as linear by default so we ignore gamma here
                            alphaTextureName, alphaTextureString = makePbrtTexture(alphaTextureFile, 'alpha', 'float', False)
                        except (RuntimeError, ValueError) as err:
                            logger.error(err)
                        else:
                            pbrtMaterials.write(alphaTextureString + '\n')

                bumpTextureName = None
                if material[c4d.MATERIAL_USE_BUMP]:
                    bumpShader = material[c4d.MATERIAL_BUMP_SHADER]
                    if bumpShader and bumpShader.GetType() == c4d.Xbitmap:
                        try:
                            bumpTextureFile = ManageTexture(bumpShader[c4d.BITMAPSHADER_FILENAME], doc, pbrtDir, None)
                            bumpTextureName, bumpTextureString = makePbrtTexture(bumpTextureFile, 'bump', 'float', False)
                        except (RuntimeError, ValueError) as err:
                            logger.error(err)
                        else:
                            pbrtMaterials.write(bumpTextureString + '\n')

                glossyColor = c4d.Vector(0.0)
                reflectivityColor = c4d.Vector(0.0)
                roughness = 0.1
                ior = 1.333
                specularTextureName = None
                if material[c4d.MATERIAL_USE_REFLECTION]:
                    reflectDataId = c4d.REFLECTION_LAYER_LAYER_DATA + c4d.REFLECTION_LAYER_LAYER_SIZE *  4 # 4 = default layer id
                    roughness = material[reflectDataId + c4d.REFLECTION_LAYER_MAIN_VALUE_ROUGHNESS]
                    specularColor = material[reflectDataId + c4d.REFLECTION_LAYER_COLOR_COLOR]
                    specularColorBrightness = material[reflectDataId + c4d.REFLECTION_LAYER_COLOR_BRIGHTNESS]
                    specularStrength = material[reflectDataId + c4d.REFLECTION_LAYER_MAIN_VALUE_SPECULAR]

                    reflectType = material[reflectDataId + c4d.REFLECTION_LAYER_MAIN_DISTRIBUTION]
                    reflectionStrength = 0.25
                    if reflectType == c4d.REFLECTION_DISTRIBUTION_GGX:
                        reflectionStrength = material[reflectDataId + c4d.REFLECTION_LAYER_MAIN_VALUE_REFLECTION]
                        if material[reflectDataId + c4d.REFLECTION_LAYER_FRESNEL_MODE] == c4d.REFLECTION_FRESNEL_DIELECTRIC:
                            ior = material[reflectDataId + c4d.REFLECTION_LAYER_FRESNEL_VALUE_IOR]
                    elif reflectType == c4d.REFLECTION_DISTRIBUTION_SPECULAR_BLINN:
                        reflectionStrength = material[reflectDataId + c4d.REFLECTION_LAYER_MAIN_VALUE_SPECULAR]

                    glossyColor = specularColor * specularColorBrightness * specularStrength
                    reflectivityColor = specularColor * specularColorBrightness * reflectionStrength

                    specularColorShader = material[reflectDataId + c4d.REFLECTION_LAYER_COLOR_TEXTURE]
                    if specularColorShader is not None and specularColorShader.GetType() == c4d.Xbitmap:
                        specularTextureFile = ManageTexture(specularColorShader[c4d.BITMAPSHADER_FILENAME], doc, pbrtDir, None)
                        try:
                            specularTextureName, specularTextureString = makePbrtTexture(specularTextureFile, 'specular')
                        except RuntimeError as err:
                            logger.error(err)
                        else:
                            pbrtMaterials.write(specularTextureString + '\n')

                useTranslucency = False
                if material[c4d.MATERIAL_USE_LUMINANCE]:
                    luminanceShader = material[c4d.MATERIAL_LUMINANCE_SHADER]
                    if luminanceShader is not None:
                        if luminanceShader.GetType() == c4d.Xfusion:
                            if luminanceShader[c4d.SLA_FUSION_BASE_CHANNEL] is not None:
                                if luminanceShader[c4d.SLA_FUSION_BASE_CHANNEL].GetType() == c4d.Xtranslucency:
                                    useTranslucency = True
                        elif luminanceShader.GetType() == c4d.Xlayer:
                            shader = luminanceShader.GetDown()
                            while shader != None:
                                if shader.GetType() == c4d.Xtranslucency:
                                    useTranslucency = True
                                    break
                                shader = shader.GetNext()

                opacityColor = c4d.Vector(1.0)
                if material[c4d.MATERIAL_USE_TRANSPARENCY]:
                    opacityColor = (material[c4d.MATERIAL_TRANSPARENCY_COLOR] * material[c4d.MATERIAL_TRANSPARENCY_BRIGHTNESS])
                transmissivityColor = c4d.Vector(1.0) - opacityColor

                # create named material if material is translucent so we can reference it from Mix material
                if useTranslucency:
                    prefix = 'MakeNamedMaterial "' + material.GetName() + '_front" "string type"'
                else:
                    prefix = 'MakeNamedMaterial "' + material.GetName() + '" "string type"'

                if not c4d.utils.CompareFloatTolerant(transmissivityColor.GetLength(), 0.0):
                    pbrtMaterials.write(prefix +' "glass" ' + makePbrtAttributeFloat('index', ior))
                    pbrtMaterials.write(' ' + makePbrtAttribute('Kr', reflectivityColor if specularTextureName is None else specularTextureName))
                    pbrtMaterials.write(' ' + makePbrtAttribute('Kt', transmissivityColor, False))
                else:
                    pbrtMaterials.write(prefix +' "uber" ' + makePbrtAttributeFloat('index', ior))
                    pbrtMaterials.write(' ' + makePbrtAttribute('Kd', baseColor if colorTextureName is None else colorTextureName))
                    pbrtMaterials.write(' ' + makePbrtAttribute('Ks', glossyColor if specularTextureName is None else specularTextureName))
                    pbrtMaterials.write(' ' + makePbrtAttribute('Kr', reflectivityColor if specularTextureName is None else specularTextureName))
                    pbrtMaterials.write(' ' + makePbrtAttributeFloat('roughness', roughness))
                    #pbrtMaterials.write(' ' + makePbrtAttributeColor('opacity', opacityColor, False))

                if useTranslucency:
                    # create translucent material
                    pbrtMaterials.write('\n')
                    pbrtMaterials.write('MakeNamedMaterial "' + material.GetName() + '_back" "string type" "translucent" "rgb reflect" [0.0 0.0 0.0] "rgb transmit" [1.0 1.0 1.0] ')
                    pbrtMaterials.write(' ' + makePbrtAttribute('Kd', baseColor if colorTextureName is None else colorTextureName))
                    # create mix material
                    pbrtMaterials.write('\n')
                    pbrtMaterials.write('Material "mix" "color amount" [0.4 0.4 0.4] "string namedmaterial1" "' + material.GetName() + '_front" "string namedmaterial2" "' + material.GetName() + '_back"')

                # all materials take a texture that can be used to specify a bump map
                # (commented this line because it produces a warning when used with mixed material and duplicated it instead whereever it s used)
                if bumpTextureName:
                   pbrtMaterials.write(' ' + makePbrtAttribute('bumpmap', bumpTextureName))
                pbrtMaterials.write('\n')

        # these are the point, uv, normal and index lists to be filled 
        # and written to the upcoming triangle mesh
        points = []
        uvs = None
        if (uvwTag):
            uvs = []
        normals = None
        if tmpPhongNormals:
            normals = []
        indices = []


        # for each original vertex index store a list of target vertices. With these vertices we store the new index, the uv texture coordinate and the normal
        vertexMap = dict()
        for iPolygon, polygon in zip(range(obj.GetPolygonCount()), obj.GetAllPolygons()):
            if (currentSelection and not currentSelection.IsSelected(iPolygon)) or touchedPolygons.IsSelected(iPolygon):
                continue

            touchedPolygons.Select(iPolygon)

            # add this polygon to the list of touched polygons

            # for each vertex, get the index, uv coordinate and phong normal and check if it already exists
            if uvwTag:
                uvwdict = uvwTag.GetSlow(iPolygon)
            else:
                uvwdict = {"a" : None, "b" : None, "c" : None, "d" : None}

            # helper function that checks whether a point that references the same original mesh
            # point and has the same texture coordinates and normal already exists. If it exists,
            # its index is returned. If it doesn't, it is created and the new index is returned.
            # This is necessary, because pbrt stores normals and texture coordinates per point and
            # never per vertex, so we effectively have to split up points that have discontinuous
            # texture coordinates or normals into multiple points.
            def checkVertex(index, uvw, normal):
                newIndex = len(points)
                if vertexMap.has_key(index):
                    for entry in vertexMap[index]:
                        if (not uvw or (math.fabs(entry[0].x - uvw.x)    < EPSILON and math.fabs(entry[0].y - uvw.y)    < EPSILON and math.fabs(entry[0].z - uvw.z)    < EPSILON)) and \
                           (not normal or (math.fabs(entry[1].x - normal.x) < EPSILON and math.fabs(entry[1].y - normal.y) < EPSILON and math.fabs(entry[1].z - normal.z) < EPSILON)):
                            return entry[2]

                    vertexMap[index].append([uvw, normal, newIndex])
                else:
                    vertexMap[index] = [[uvw, normal, newIndex]]

                # we only get here if there is no matching entry for the new vertex already, so go ahead and make one
                # store a new vertx entry, since there is either no entry for this index or uvw or normal were different
                points.append(obj.GetPoint(index))
                if uvw != None and uvs != None:
                    uvs.append(uvw)
                if normals != None:
                    normals.append(normal)

                return newIndex

            # vertex a
            newIndexA = checkVertex(polygon.a, uvwdict["a"], None if not tmpPhongNormals else tmpPhongNormals[iPolygon*4+0])
            indices.append(newIndexA)

            # vertex b
            indices.append(checkVertex(polygon.b, uvwdict["b"], None if not tmpPhongNormals else tmpPhongNormals[iPolygon*4+1]))

            # vertex c
            newIndexC = checkVertex(polygon.c, uvwdict["c"], None if not tmpPhongNormals else tmpPhongNormals[iPolygon*4+2])
            indices.append(newIndexC)

            if polygon.c != polygon.d:
                indices.append(newIndexC)

                # vertex d
                indices.append(checkVertex(polygon.d, uvwdict["d"], None if not tmpPhongNormals else tmpPhongNormals[iPolygon*4+3]))

                indices.append(newIndexA)

        if len(indices) > 0:
            pbrtGeometry.write(indent + 'Shape "trianglemesh"  "integer indices" [')
            # write indices
            for index in indices:
                pbrtGeometry.write(str(index) + ' ')
            # write points
            pbrtGeometry.write('] "point P" [')
            #for point in obj.GetAllPoints():
            for point in points:
                pbrtGeometry.write(str(point.x) + ' ' + str(point.y) + ' ' + str(point.z) + ' ')
            # write uv coordinates, apply texture tag scaling settings
            if uvs:
                offsetx = textureTag[c4d.TEXTURETAG_OFFSETX]
                offsety = textureTag[c4d.TEXTURETAG_OFFSETY]
                lengthx = textureTag[c4d.TEXTURETAG_LENGTHX]
                lengthy = textureTag[c4d.TEXTURETAG_LENGTHY]
                pbrtGeometry.write('] "float uv" [')
                for uv in uvs:
                    pbrtGeometry.write(str((uv.x - offsetx) / lengthx) + ' ' + str((-uv.y + offsety) / lengthy) + ' ')
            # write normals
            if normals:
                pbrtGeometry.write('] "normal N" [')
                for normal in normals:
                    pbrtGeometry.write(str(normal.x) + ' ' + str(normal.y) + ' ' + str(normal.z) + ' ')
            # close array
            pbrtGeometry.write(']')
            if alphaTextureName:
                pbrtGeometry.write('"texture alpha" "' + alphaTextureName + '"')
            pbrtGeometry.write('\n')

        if innerAttributeBlock:
            indent = indent[0:-1]
            pbrtGeometry.write(indent + 'AttributeEnd\n')

        if touchedPolygons.GetCount() == obj.GetPolygonCount():
            break


#
# Run through the scene recursively and call the passed function for every object.
# The passed function is supposed to return a bool, when that bool is False, it should not walk
# into the object that was just processed.
# The function will stack AttributeBegin/AttributeEnd blocks like they are in the scene hierarchy
# The function expects an c4d.BaseObject or c4d.BaseDocument as second parameter
#
# The root object is mainly passed to stop traversing up the tree for visibility detection
# this is important when writing object trees which are instanced and therefore don't take
# upstream visibility settings into account
#
def WalkObjectTree(pbrtGeometry, pbrtMaterials, obj, exportedMaterials, functionToCall, namedObjects = [], indent = "", rootObj = None):
    if type(obj) != c4d.documents.BaseDocument:
        # call the function that does the actual thing
        # when this function returns False, no further traversal into children should be done
        result = functionToCall(pbrtGeometry, pbrtMaterials, obj, exportedMaterials, namedObjects, indent, rootObj)
        if result:
            cache = obj.GetDeformCache()
            if cache == None:
                cache = obj.GetCache()
            while cache:
                pbrtGeometry.write(indent + "AttributeBegin\n")
                indent += "\t"
                pbrtGeometry.write(indent + "# " + cache.GetName() + "\n")
                ml = cache.GetMl()
                if ml != c4d.Matrix():
                    pbrtGeometry.write(indent + 'ConcatTransform [' + str(ml.v1.x) + ' ' + str(ml.v1.y) + ' ' + str(ml.v1.z) + ' 0  ' + str(ml.v2.x) + ' ' + str(ml.v2.y) + ' ' + str(ml.v2.z) + ' 0  ' + str(ml.v3.x) + ' ' + str(ml.v3.y) + ' ' + str(ml.v3.z) + ' 0  ' + str(ml.off.x) +  ' ' + str(ml.off.y) + ' ' + str(ml.off.z) + ' 1]\n')
                WalkObjectTree(pbrtGeometry, pbrtMaterials, cache, exportedMaterials, functionToCall, namedObjects, indent, rootObj)
                indent = indent[0:-1]
                pbrtGeometry.write(indent + "AttributeEnd\n")
                cache = cache.GetNext()
        
            for child in obj.GetChildren():
                pbrtGeometry.write(indent + "AttributeBegin\n")
                indent += "\t"
                pbrtGeometry.write(indent + "# " + child.GetName() + "\n")
                ml = child.GetMl()
                if ml != c4d.Matrix():
                    pbrtGeometry.write(indent + 'ConcatTransform [' + str(ml.v1.x) + ' ' + str(ml.v1.y) + ' ' + str(ml.v1.z) + ' 0  ' + str(ml.v2.x) + ' ' + str(ml.v2.y) + ' ' + str(ml.v2.z) + ' 0  ' + str(ml.v3.x) + ' ' + str(ml.v3.y) + ' ' + str(ml.v3.z) + ' 0  ' + str(ml.off.x) +  ' ' + str(ml.off.y) + ' ' + str(ml.off.z) + ' 1]\n')
                WalkObjectTree(pbrtGeometry, pbrtMaterials, child, exportedMaterials, functionToCall, namedObjects, indent, rootObj)
                indent = indent[0:-1]
                pbrtGeometry.write(indent + "AttributeEnd\n")

    else:
        for o in obj.GetObjects():
            pbrtGeometry.write(indent + "AttributeBegin\n")
            indent += "\t"
            pbrtGeometry.write(indent + "# " + o.GetName() + "\n")
            ml = o.GetMl()
            if ml != c4d.Matrix():
                pbrtGeometry.write(indent + 'ConcatTransform [' + str(ml.v1.x) + ' ' + str(ml.v1.y) + ' ' + str(ml.v1.z) + ' 0  ' + str(ml.v2.x) + ' ' + str(ml.v2.y) + ' ' + str(ml.v2.z) + ' 0  ' + str(ml.v3.x) + ' ' + str(ml.v3.y) + ' ' + str(ml.v3.z) + ' 0  ' + str(ml.off.x) +  ' ' + str(ml.off.y) + ' ' + str(ml.off.z) + ' 1]\n')
            WalkObjectTree(pbrtGeometry, pbrtMaterials, o, exportedMaterials, functionToCall, namedObjects, indent, rootObj)
            indent = indent[0:-1]
            pbrtGeometry.write(indent + "AttributeEnd\n")

def IsProcessRunning():
    global g_thread
    return g_thread is not None and g_thread.IsRunning()

def TerminateProcess():
    global g_thread
    if g_thread is not None:
        g_thread.TerminateRenderer()
        g_thread.End()
        g_thread = None

def GetRenderModeRec(obj, rootObj=None):
    renderMode = obj.GetRenderMode()
    if renderMode is c4d.MODE_UNDEF and obj != rootObj:
        parent = obj.GetUp()
        if parent:
            renderMode = GetRenderModeRec(parent, rootObj)
        else:
            parent = obj.GetCacheParent()
            if parent:
                renderMode = GetRenderModeRec(parent, rootObj)

    return renderMode


class PbrtThread(c4d.threading.C4DThread):
    def __init__(self, doc, mode, data, exe, path):
        super(c4d.threading.C4DThread, self).__init__()
        self.doc = doc.GetClone(c4d.COPYFLAGS_DOCUMENT)
        self.mode = mode
        self.data = data
        self.exe = exe
        self.path = path
        self.pbrtProcess = None
       

    def Main(self):
        # reset global variables
        global g_bakeDoc
        global g_bakeTag
        global g_bakeTextureFile
        global g_nLightSourcesExported
        g_bakeDoc = None
        g_bakeTag = None
        g_bakeTextureFile = None
        g_nLightSourcesExported = 0

        logger.info("Evaluating Document Copy...")
        self.doc.ExecutePasses(self.Get(), True, True, True, c4d.BUILDFLAGS_EXTERNALRENDERER)

        logger.info("Starting Export...")
        pbrtFilename, imageFilename = self.ExportDocumentToPbrt(self.doc, self.data, self.path)

        # if an environment file was created...
        if g_bakeTextureFile:
            # ...first wait until the file actually shows up...
            for i in range(60):
                if os.path.exists(g_bakeTextureFile):
                    break
                logger.info("Waiting for Environment Baking to finish...")
                time.sleep(1)

            # ...then wait until it is can be opened for reading
            for i in range(240):
                try:
                    bakeFileTmp = open(g_bakeTextureFile, 'r')
                    bakeFileTmp.close()
                    break
                except:
                    logger.info("Waiting for Environment Baking to finish...")
                    time.sleep(1)

            # also don't forget to free the tag and document
            g_bakeTag = None
            g_bakeDoc = None

        # if an output filename is defined and the gui is set to start the rendering, go ahead
        if imageFilename is not None and self.mode != IDC_PBRT_MODE_EXPORT:
            global g_bmp
            logger.info("Start External Rendering...")
            g_bmp = self.ExecuteRenderer(pbrtFilename, imageFilename)

        c4d.SpecialEventAdd(MSG_PBRT_FINISHED)


    #
    # export the current document as pbrt file, return the path of the resulting file and the output image file
    #
    def ExportDocumentToPbrt(self, doc, data, dest=""):
        bc = doc.GetActiveRenderData().GetData()

        docFilename = doc.GetDocumentName()
        docPath = doc.GetDocumentPath()
        globalLightScale = data.GetFloat(IDC_PBRT_INTENSITY)

        # generate a temporary location and filename for the exported file image
        if len(dest) > 0:
            pbrtFilename = dest
        else:
            pbrtFilename = os.path.join(tempfile.gettempdir(), os.path.splitext(docFilename)[0] + ".pbrt")

        pbrt = open(pbrtFilename, 'w')

        pbrtMaterialsFilename = os.path.splitext(pbrtFilename)[0] + '_materials' + os.path.splitext(pbrtFilename)[1]
        pbrtGeometryFilename = os.path.splitext(pbrtFilename)[0] + '_geometry' + os.path.splitext(pbrtFilename)[1]
            
        #
        # output scene options that are derived from the camera/viewport
        #
        camera = doc.GetRenderBaseDraw()
        mi = camera.GetMi()
        cameraObject = camera.GetSceneCamera(doc)
        if (cameraObject):
            mi = ~cameraObject.GetMg()
        pbrt.write('Transform [' + str(mi.v1.x) + ' ' + str(mi.v1.y) + ' ' + str(mi.v1.z) + ' 0  ' + str(mi.v2.x) + ' ' + str(mi.v2.y) + ' ' + str(mi.v2.z) + ' 0  ' + str(mi.v3.x) + ' ' + str(mi.v3.y) + ' ' + str(mi.v3.z) + ' 0  ' + str(mi.off.x) +  ' ' + str(mi.off.y) + ' ' + str(mi.off.z) + ' 1]\n')

        # we need render settings for dof, so they are initialized here already
        renderData = doc.GetActiveRenderData()
        renderDataBc = renderData.GetDataInstance()

        # Pbrt expects the camera fov to be the spread angle of the viewing frustum along the narrower
        # of the images width and height. CINEMA 4D provides both, so we just grab the smaller of the
        # two.
        cameraFov         = cameraObject[c4d.CAMERAOBJECT_FOV]
        cameraVerticalFov = cameraObject[c4d.CAMERAOBJECT_FOV_VERTICAL]
        if cameraFov > cameraVerticalFov:
            cameraFov = cameraVerticalFov
        if cameraObject == None:
            pbrt.write('Camera "perspective" "float fov" [' + str(c4d.utils.Deg(cameraFov)) + ']\n')
        else:
            focalLength = cameraObject.GetDataInstance()[c4d.CAMERAOBJECT_TARGETDISTANCE]
            lensRadius = 0 # default to pinhole camera
            vp = renderData.GetFirstVideoPost()
            while vp:
                if vp.GetType() == 1023342 and not vp.GetBit(c4d.BIT_VPDISABLED) and vp.GetDataInstance()[c4d.VP_XMB_DOF]:
                    # Formula to compute the lens diameter from f-stop and focal length from here:
                    # http://www.punitsinha.com/resource/aperture_focal_length.html
                    # consider document scale, because focal length is always given in mm
                    # divide result by 2 because pbrt expects lens radius
                    mmUnitScale = c4d.UnitScaleData()
                    mmUnitScale.SetUnitScale(1, c4d.DOCUMENT_UNIT_MM)
                    scale = c4d.utils.CalculateTranslationScale(mmUnitScale, doc.GetSettingsInstance(c4d.DOCUMENTSETTINGS_DOCUMENT)[c4d.DOCUMENT_DOCUNIT])
                    lensRadius = (cameraObject.GetDataInstance()[c4d.CAMERA_FOCUS] * scale) / cameraObject.GetDataInstance()[c4d.CAMERAOBJECT_FNUMBER_VALUE] / 2
                vp = vp.GetNext()
            pbrt.write('Camera "perspective" "float fov" [' + str(c4d.utils.Deg(cameraFov)) + '] "float focaldistance" [' + str(focalLength) + '] "float lensradius" [' + str(lensRadius) + ']\n')

        #
        # output scene options that are derived from the render settings
        #
        pbrt.write('Sampler "halton" "integer pixelsamples" [' + str(data.GetInt32(IDC_PBRT_SAMPLES)) + ']\n')
        
        # generate a temporary location and filename for the output image
        # TODO: Also make the filename random
        if len(dest) > 0:
            destFile = os.path.basename(dest)
            imageFilename = os.path.splitext(destFile)[0] + '.exr'
        else:
            imageFilename = os.path.join(tempfile.gettempdir(), 'simple.exr')
        pbrt.write('Film "image" "string filename" ["' + imageFilename.replace('\\','\\\\') + '"] "integer xresolution" [' + str(int(renderDataBc[c4d.RDATA_XRES])) +'] "integer yresolution" [' + str(int(renderDataBc[c4d.RDATA_YRES])) + ']\n')
        
        # search for a global illumination post effect to decide which surfaceintegrator to use
        renderVideoPost = renderData.GetFirstVideoPost()
        while renderVideoPost:
            if renderVideoPost.GetType() == 1021096 and not renderVideoPost.GetBit(c4d.BIT_VPDISABLED):
                logger.info('Using Integrator "path" Due To Global Illumination Effect Present.')
                pbrt.write('Integrator "path" "integer maxdepth" [5]\n')
                break
            renderVideoPost = renderVideoPost.GetNext()

        if not renderVideoPost:
            logger.info('Using Integrator "directlighting" Due To Global Illumination Effect Not Present.')
            pbrt.write('Integrator "directlighting" "integer maxdepth" [5] "string strategy" "all"\n')

        pbrt.write('WorldBegin\n')
        indent = "\t"

        #
        # set default material
        #
        pbrt.write(indent + '# Default Material\n')
        defaultMaterialColor = doc[c4d.DOCUMENT_DEFAULTMATERIAL_COLOR]
        linearDefaultMaterialColor = c4d.utils.TransformColor(defaultMaterialColor, c4d.COLORSPACETRANSFORMATION_SRGB_TO_LINEAR)
        pbrt.write(indent + 'Material "uber" "rgb Kd" [' + str(linearDefaultMaterialColor.x) + ' ' + str(linearDefaultMaterialColor.y) + ' ' + str(linearDefaultMaterialColor.z) +'] "float index" [1.333]\n')
        
        """
        define an export function to be passed to WalkObjectTree as callback
        TODO: should identify (physical) sky objects, bake them to a texture and define them as a light source
        """
        def MyExportFunction(pbrtGeometry, pbrtMaterials, obj, exportedMaterials, namedObjects = [], indent="", rootObj=None):
            # walk through all the possible reasons to not export this object
            # if a reason is found, still return true, because this doesn't mean the subtree shouldn't be traversed
            if not obj.GetDeformMode():
                return True
            
            renderMode = GetRenderModeRec(obj, rootObj)
            if renderMode == c4d.MODE_OFF:
                return True

            # take care of render instances and instanced objcts
            if obj.GetType() == c4d.Oinstance and obj[c4d.INSTANCEOBJECT_RENDERINSTANCE] == True:
                # make sure this instance is not used as input object for generator (LAUBWERKCINEMA-769)
                # while this flag is always true for normal instances this check will work for render instances
                if not obj.GetBit(c4d.BIT_CONTROLOBJECT) and not obj.GetBit(c4d.BIT_IGNOREDRAW):
                    namedObject = FindNamedObject(obj.GetDataInstance().GetObjectLink(c4d.INSTANCEOBJECT_LINK), namedObjects)
                    if namedObject:
                        pbrtGeometry.write(indent + 'ObjectInstance "' + namedObject[1] + '"\n')
                # if this object has been inserted as an object instance, we return False to notify WalkObjectTree that it should not traverse the hierarchy below this object
                return False
            elif FindNamedObject(obj, namedObjects):
                namedObject = FindNamedObject(obj, namedObjects)
                if namedObject:
                    pbrtGeometry.write(indent + 'ObjectInstance "' + namedObject[1] + '"\n')
                # if this object has been inserted as an object instance, we return False to notify WalkObjectTree that it should not traverse the hierarchy below this object
                return False

            # this needs to be checked after the check whether this is an instanced object, 
            # because generator objects also return True here
            if obj.GetBit(c4d.BIT_CONTROLOBJECT):
                return True

            # this object is visible, write it to file
            if obj.GetType() == c4d.Olight:
                global g_nLightSourcesExported
                g_nLightSourcesExported += 1
                # export this object as a light
                lightColor = obj[c4d.LIGHT_COLOR]
                lightBrightness = obj[c4d.LIGHT_BRIGHTNESS]
                #lightBrightness *= 100
                if obj[c4d.LIGHT_TYPE] == c4d.LIGHT_TYPE_OMNI:
                    lightScale = obj[c4d.LIGHT_DETAILS_OUTERRADIUS] * globalLightScale
                    pbrtGeometry.write(indent + 'LightSource "point" "rgb I" [' + str(lightColor.x * lightBrightness) + ' ' + str(lightColor.y * lightBrightness) + ' ' + str(lightColor.z * lightBrightness) + '] "point from" [0 0 0] "rgb scale" [' + str(lightScale) + ' ' + str(lightScale) + ' ' + str(lightScale) + '] \n')
                elif obj[c4d.LIGHT_TYPE] == c4d.LIGHT_TYPE_SPOT:
                    pass
                elif obj[c4d.LIGHT_TYPE] == c4d.LIGHT_TYPE_DISTANT:
                    pbrtGeometry.write(indent + 'LightSource "distant" "rgb L" [' + str(lightColor.x * lightBrightness) + ' ' + str(lightColor.y * lightBrightness) + ' ' + str(lightColor.z * lightBrightness) + '] "point from" [0 0 0] "point to" [0 0 1]')
                    if not c4d.utils.CompareFloatTolerant(globalLightScale, 1.0):
                        pbrtGeometry.write(' ' + makePbrtAttributeColor('scale', c4d.Vector(globalLightScale), False))
                    pbrtGeometry.write('\n')
                else:
                    logger.warning("Light Source " + obj.GetName() + " ignored due to unknown Light Type Setting!")

            elif obj.GetType() == c4d.Osky:
                # in case of a sky object present, we copy that object into a separate scene, bake it into a texture and apply that texture as infinite light to the scene
                # see LAUBWERKCINEMA-697
                parent = obj.GetCacheParent()
                #print parent
                #print parent.GetName()
                #print parent.GetType()
                if parent and parent.GetType() == 1011146:

                    global g_bakeTextureFile
                    if not g_bakeTextureFile:
                        global g_bakeDoc
                        g_bakeDoc = c4d.documents.IsolateObjects(parent.GetDocument(), [parent])
                        if c4d.documents.MergeDocument(doc=g_bakeDoc, name=os.path.join(os.path.dirname(__file__), "res", "pbrt-env-bake.c4d"), loadflags=c4d.SCENEFILTER_OBJECTS|c4d.SCENEFILTER_MATERIALS, thread=self.Get()):
                            
                            #c4d.documents.SaveDocument(doc=g_bakeDoc, name='C:\\bakeDoc.c4d', saveflags=c4d.SAVEDOCUMENTFLAGS_0, format=c4d.FORMAT_C4DEXPORT)
                            
                            global g_bakeTag
                            g_bakeTag = g_bakeDoc.GetFirstObject().GetFirstTag()
                            
                            # set the output file in the bake tag, so the texture gets written to the correct location
                            bakeTagTexturename = os.path.join(os.path.dirname(pbrtFilename), "environment_")
                            g_bakeTag[c4d.BAKETEXTURE_NAME] = bakeTagTexturename

                            # the tag will append "Reflection" and the extension
                            # store the filename globally, so we can later check how it's progressing
                            g_bakeTextureFile = os.path.join(os.path.dirname(pbrtFilename), "environment_Reflection.exr")

                            # if the file already exists, remove it to prevent the bake command to trigger a dialog popup
                            if os.path.exists(g_bakeTextureFile):
                                try:
                                    os.remove(g_bakeTextureFile)
                                except:
                                    os.error("Unable to remove previous baked environment texture!")

                            # trigger the texture bake event, so the environment texture gets written
                            # we have to use a message, because we can't call into another thread directly
                            c4d.SpecialEventAdd(MSG_PBRT_BAKETEXTURE)

                            # rotate the light so it matches the CINEMA 4D Physical Sky environment
                            pbrtGeometry.write(indent + 'Rotate 90 -1 0 0\n')
                            pbrtGeometry.write(indent + 'Scale 1 -1 1\n')

                            # write the infinite light to the scene
                            pbrtGeometry.write(indent + 'LightSource "infinite" "string mapname" ["' + os.path.basename(g_bakeTextureFile) +  '"] "color L" [1 1 1] "integer nsamples" [128]')
                            if not c4d.utils.CompareFloatTolerant(globalLightScale, 1.0):
                                pbrtGeometry.write(' ' + makePbrtAttributeColor('scale', c4d.Vector(globalLightScale), False))
                            pbrtGeometry.write('\n')
                        else:
                            logger.error("Unable to merge environment bake template!")

                    else:
                        logger.warning("More than one Physical Sky object found for baking, ignored!")

            elif obj.GetType() == c4d.Opolygon:
                # this is a regular polygon object, export as such
                logger.debug("Exporting Polygon Object: " + obj.GetName())
                ExportPolygonObject(pbrtGeometry, pbrtMaterials, exportedMaterials, obj, indent)

            return True


        #
        # define named objects
        #
        namedObjects = []
        for obj in iter_all(doc):
            if self.TestBreak():
                return None, None

            # find visible render instance objects
            if obj.GetType() == c4d.Oinstance:
                if not obj[c4d.INSTANCEOBJECT_RENDERINSTANCE]:
                    continue

                if GetRenderModeRec(obj, None) != c4d.MODE_OFF:
                    instancedObject = obj.GetDataInstance().GetObjectLink(c4d.INSTANCEOBJECT_LINK)
                    if instancedObject and not FindNamedObject(instancedObject, namedObjects):
                        # make the name of the named object
                        objectName = SanitizeObjectName(instancedObject.GetName())
                        # append to list of named objects
                        namedObjects.append([instancedObject, objectName])

        # create separate file for content
        pbrtMaterials = open(pbrtMaterialsFilename, 'w')
        pbrtGeometry = open(pbrtGeometryFilename, 'w')
        exportedMaterials = dict()
        for namedObject in namedObjects:
            pbrtGeometry.write('ObjectBegin "' + namedObject[1] + '"\n')
            logger.info("Exporting " + namedObject[0].GetName() + " as Instance...")
            WalkObjectTree(pbrtGeometry, pbrtMaterials, namedObject[0], exportedMaterials, MyExportFunction, indent=indent, rootObj=namedObject[0])
            pbrtGeometry.write('ObjectEnd\n')

        # main scene export run
        WalkObjectTree(pbrtGeometry, pbrtMaterials, doc, exportedMaterials, MyExportFunction, namedObjects)
        pbrtMaterials.close()
        pbrtGeometry.close()

        # in case of no light sources in the scene and the corresponding option being set,
        # export default light
        global g_nLightSourcesExported
        if renderData[c4d.RDATA_AUTOLIGHT] and g_nLightSourcesExported == 0:
            lightVector =  (~mi).MulV(-camera[c4d.BASEDRAW_DATA_LIGHTVECTOR])
            pbrt.write(indent + 'LightSource "distant" "rgb L" [1 1 1] "point from" [0 0 0] "point to" [' + str(lightVector.x) + ' ' + str(lightVector.y) + ' ' + str(lightVector.z) + ']')
            if not c4d.utils.CompareFloatTolerant(globalLightScale, 1.0):
                pbrt.write(' ' + makePbrtAttributeColor('scale', c4d.Vector(globalLightScale), False))
            pbrt.write('\n')

        pbrt.write(indent + 'Include "' + os.path.basename(pbrtMaterialsFilename) + '"\n');
        pbrt.write(indent + 'Include "' + os.path.basename(pbrtGeometryFilename) + '"\n');

        """
        pbrt.write('AttributeBegin\n')
        pbrt.write('  Rotate 135 1 0 0\n')
        pbrt.write('  Texture "checks" "spectrum" "checkerboard" "float uscale" [4] "float vscale" [4] "rgb tex1" [1 0 0] "rgb tex2" [0 0 1]\n')
        pbrt.write('  Material "matte" "texture Kd" "checks"\n')
        pbrt.write('  Shape "disk" "float radius" [20] "float height" [-1]\n')
        pbrt.write('AttributeEnd\n')
        """

        indent = ""
        pbrt.write('WorldEnd\n')
        
        pbrt.close()
        
        return pbrtFilename, imageFilename

    def TerminateRenderer(self):
        if self.pbrtProcess is not None and self.pbrtProcess.poll() is None:
            self.pbrtProcess.terminate()

    #
    # executes the actual renderer and returns the rendered image
    #
    def ExecuteRenderer(self, pbrtFilename, imageFilename):
        # call pbrt for rendering
        try:
            self.pbrtProcess = subprocess.Popen([os.path.join(os.path.dirname(__file__), 'pbrt', 'bin', 'pbrt.exe'), pbrtFilename], cwd=os.path.dirname(pbrtFilename))
        except subprocess.CalledProcessError as err:
            logger.error("Failed to render image: " + str(err))
            return None
        else:
            # wait until rendering is finished or user aborts the process
            self.pbrtProcess.wait()

            # check if the return code of the renderer indicates that it ran through without error
            bmp = None
            if self.pbrtProcess.poll() == 0:
                # display resulting image
                bmp = c4d.bitmaps.BaseBitmap()
                result, ismovie = bmp.InitWith(imageFilename)

            # if we're only rendering, clean up the files
            if self.data.GetInt32(IDC_PBRT_MODE) == IDC_PBRT_MODE_RENDER:
                # remove exported pbrt file
                try:
                    os.remove(pbrtFilename)
                except OSError as err:
                    logger.error('Unable to remove "' + pbrtFilename + '": ' + str(err))

                # if the image wasn't supposed to be saved, the temp image file can now be deleted
                try:
                    if os.path.exists(imageFilename):
                        os.remove(imageFilename)
                except OSError as err:
                    logger.error('Unable to remove "' + imageFilename + '": ' + str(err))

            return bmp


class ExportToPbrtDialog(c4d.gui.GeDialog):
    def __init__(self):
        super(c4d.gui.GeDialog, self).__init__()
        self._data = None
        self.treeview = None
        self.lastScrollPos = 0
        self.autoscroll = True

    # called when the dialog is opened - load or generate the GUI here
    def CreateLayout(self):
        res = self.LoadDialogResource(DLG_PBRT)
        return res

    @property
    def data(self):
        self._data = c4d.plugins.GetWorldPluginData(PBRT_EXPORT_ID)
        if self._data is None:
            self._data = c4d.BaseContainer()
            self._data.SetFilename(IDC_PBRT_EXE, "")
            self._data.SetInt32(IDC_PBRT_MODE, IDC_PBRT_MODE_RENDER)
            self._data.SetInt32(IDC_PBRT_SAMPLES, 32)
            self._data.SetFloat(IDC_PBRT_INTENSITY, 1.0)
            self._data.SetInt32(IDC_PBRT_LOGLEVEL, IDC_PBRT_LOGLEVEL_INFO)
            c4d.plugins.SetWorldPluginData(PBRT_EXPORT_ID, self._data)
        return self._data

    def EnableControls(self, enable):
        self.Enable(IDC_PBRT_MODE, enable)
        self.Enable(IDC_PBRT_EXE, enable)
        self.Enable(IDC_PBRT_SAMPLES, enable)
        self.Enable(IDC_PBRT_INTENSITY, enable)

    def UpdateGui(self):
        self.LayoutFlushGroup(IDC_BUTTON_GROUP)

        if not IsProcessRunning():
            self.EnableControls(True)
            self.AddButton(IDC_PBRT_START, c4d.BFH_LEFT|c4d.BFH_SCALEFIT, name=c4d.plugins.GeLoadString(IDS_PBRT_START))
        else:
            self.EnableControls(False)
            self.AddButton(IDC_PBRT_ABORT, c4d.BFH_LEFT|c4d.BFH_SCALEFIT, name=c4d.plugins.GeLoadString(IDS_PBRT_ABORT))

        self.LayoutChanged(IDC_BUTTON_GROUP)

    def InitValues(self):
        self.SetInt32(IDC_PBRT_MODE, self.data.GetInt32(IDC_PBRT_MODE, IDC_PBRT_MODE_RENDER))
        self.SetFilename(IDC_PBRT_EXE, self.data.GetFilename(IDC_PBRT_EXE, ""))
        self.SetInt32(IDC_PBRT_SAMPLES, self.data.GetInt32(IDC_PBRT_SAMPLES, 32), 0, 4096)
        self.SetFloat(IDC_PBRT_INTENSITY, self.data.GetFloat(IDC_PBRT_INTENSITY, 1.0), 0.0, 1.0e12, 0.01, c4d.FORMAT_PERCENT)
        self.SetInt32(IDC_PBRT_LOGLEVEL, self.data.GetInt32(IDC_PBRT_LOGLEVEL, IDC_PBRT_LOGLEVEL_INFO))

        data = c4d.BaseContainer()
        data.SetInt32(TREEVIEW_COLUMN_ID_TIME,  c4d.LV_USER)
        data.SetInt32(TREEVIEW_COLUMN_ID_LEVEL, c4d.LV_USER)
        data.SetInt32(TREEVIEW_COLUMN_ID_DESCRIPTION,  c4d.LV_USER)

        self.treeview = self.FindCustomGui(IDC_PBRT_LOG, c4d.CUSTOMGUI_TREEVIEW)
        if self.treeview is None:
            return False

        self.treeview.SetLayout(3, data)
        self.treeview.SetHeaderText(TREEVIEW_COLUMN_ID_TIME, "Time")
        self.treeview.SetHeaderText(TREEVIEW_COLUMN_ID_LEVEL, "Level")
        self.treeview.SetHeaderText(TREEVIEW_COLUMN_ID_DESCRIPTION, "Description")
        self.treeview.Refresh()

        global g_errorRoot
        g_errorRoot = RootItem()
        self.treeview.SetRoot(g_errorRoot, ErrorList(self.treeview), None)

        self.UpdateGui()
        return True

    def Command(self, id, msg):
        if id == IDC_PBRT_START:
            self.autoscroll = True
            self.lastScrollPos = 0
            logger.setLevel(logging.DEBUG)
            logger.handlers = []
            logger.addHandler(ErrorHandler())
            doc = c4d.documents.GetActiveDocument()
            if doc is not None:
                mode = self.GetInt32(IDC_PBRT_MODE)
                exe = self.GetFilename(IDC_PBRT_EXE)
                if mode != IDC_PBRT_MODE_EXPORT and len(exe) == 0:
                    c4d.gui.MessageDialog("Please provide the path to the pbrt renderer executable!")
                    return False
                path = ""
                if mode != IDC_PBRT_MODE_RENDER:
                    defPath = os.path.join(doc.GetDocumentPath(), doc.GetDocumentName())
                    path = c4d.storage.SaveDialog(title="Select the destination for your export", force_suffix="pbrt", def_path=defPath)
                # check if the user did not cancel here
                if path is not None:
                    global g_thread
                    g_thread = PbrtThread(doc, mode, self.data, exe, path)
                    g_thread.Start()
                    self.UpdateGui()

            return True
        elif id == IDC_PBRT_ABORT:
            TerminateProcess()
            self.UpdateGui()
        elif id >= IDC_PBRT_EXE and id <= IDC_PBRT_SAMPLES or id == IDC_PBRT_INTENSITY:
            if id == IDC_PBRT_MODE:
                self.data.SetInt32(IDC_PBRT_MODE, self.GetInt32(IDC_PBRT_MODE))
            elif id == IDC_PBRT_EXE:
                self.data.SetFilename(IDC_PBRT_EXE, self.GetFilename(IDC_PBRT_EXE))
            elif id == IDC_PBRT_SAMPLES:
                self.data.SetInt32(IDC_PBRT_SAMPLES, self.GetInt32(IDC_PBRT_SAMPLES))
            elif id == IDC_PBRT_INTENSITY:
                self.data.SetFloat(IDC_PBRT_INTENSITY, self.GetFloat(IDC_PBRT_INTENSITY))
            elif id == IDC_PBRT_LOGLEVEL:
                self.data.SetInt32(IDC_PBRT_LOGLEVEL, self.GetInt32(IDC_PBRT_LOGLEVEL))
            c4d.plugins.SetWorldPluginData(PBRT_EXPORT_ID, self.data)
        elif id == IDC_PBRT_LOGLEVEL:
            # if the message level filter changed update the treeview
            value = self.GetInt32(IDC_PBRT_LOGLEVEL)
            global g_level
            if value == IDC_PBRT_LOGLEVEL_DEBUG:
                g_level = logging.DEBUG
            elif value == IDC_PBRT_LOGLEVEL_INFO:
                g_level = logging.INFO
            elif value == IDC_PBRT_LOGLEVEL_WARNING:
                g_level = logging.WARNING
            elif value == IDC_PBRT_LOGLEVEL_ERROR:
                g_level = logging.ERROR
            else:
                g_level = logging.INFO
            UpdateErrorList()
            UpdateTreeView()
            return True
        return False

    def CoreMessage(self, id, bc):
        if id == MSG_PBRT_UPDATE_LOG:
            if self.treeview:
                self.treeview.Refresh()
                if self.autoscroll:
                    self.treeview.MakeVisible(g_errorRoot.GetLastChild())
            return True

        elif id == MSG_PBRT_FINISHED:
            global g_thread
            if g_thread is not None:
                logger.info("Finished")

                if g_bmp is not None:
                    # it's not allowed to call ShowBitmap() in a thread
                    c4d.bitmaps.ShowBitmap(g_bmp)
            else:
                logger.error("Cancelled")
                g_thread = None
            self.UpdateGui()
            return True

        elif id == MSG_PBRT_BAKETEXTURE:
            # this triggers a bake texture tag
            # The actual baking process will run in a separate thread
            global g_bakeDoc
            global g_bakeTag
            if g_bakeTag is not None and g_bakeDoc is not None:
                c4d.CallButton(g_bakeTag, c4d.BAKETEXTURE_BAKE)
            else:
                logger.error("Bake texture thread can't find Bake Texture Tag!")
            return True

        return c4d.gui.GeDialog.CoreMessage(self, id, bc)

    def Message(self, msg, result):
        id = msg.GetId()
        if id == c4d.BFM_SCROLLGROUP_SCROLLED:
            scrollPos = msg.GetInt32(3) # vertical scroll pos
            if self.autoscroll:
                # if the user scrolled up stop scrolling down automatically
                if scrollPos > self.lastScrollPos:
                    self.autoscroll = False
            # save last scroll pos
            self.lastScrollPos = scrollPos
        return c4d.gui.GeDialog.Message(self, msg, result)


# this class is the basic plugin
class ExportToPbrtCommand(c4d.plugins.CommandData):

    _dialog = None

    @property
    def dialog(self):
        if not self._dialog:
            self._dialog = ExportToPbrtDialog()
        return self._dialog

    # we could also just execute some code here - the dialog is optional
    def Execute(self, doc):
        self.dialog.Open(c4d.DLG_TYPE_ASYNC, PBRT_EXPORT_ID, -1, -1, 260, 400)
        return True

    # needed to restore minimized dialogs and for startup layout integration
    def RestoreLayout(self, sec_ref):
        return self.dialog.Restore(PBRT_EXPORT_ID, sec_ref)

def PluginMessage(id, data):
    """Make sure to clean up when C4D ends.
    """
    if id == c4d.C4DPL_ENDACTIVITY:
        if IsProcessRunning():
            TerminateProcess()
        return True
    return False

#register the plugin
if __name__ == "__main__":
    # only load if r16 or higher (LAUBWERKCINEMA-683)
    if c4d.GetC4DVersion() >= 16000:
        # load an icon from the 'res' folder
        icon = c4d.bitmaps.BaseBitmap()
        icon.InitWith(os.path.join(os.path.dirname(__file__), "res", "icon.tif"))

        # get the plugin title from the string resource
        title = "Export to PBRT..."#c4d.plugins.GeLoadString(IDS_SUBMIT)
        # c4d.plugins.RegisterMessagePlugin(TREEVIEWUPDATETIMER_COMMAND_ID, "TreeviewUpdateTimer", 0, TreeviewUpdateTimer())
        c4d.plugins.RegisterCommandPlugin(PBRT_EXPORT_ID, title, 0, icon, title, ExportToPbrtCommand())

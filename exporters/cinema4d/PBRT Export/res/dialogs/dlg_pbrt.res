// C4D-DialogResource
DIALOG DLG_PBRT
{
    NAME IDS_PBRT;
    SCALE_V; SCALE_H;
    GROUP
    {
        SCALE_V; SCALE_H;
        COLUMNS 1;

        GROUP
        {
            SCALE_H;
            COLUMNS 2;
            BORDERSIZE 4, 4, 4, 4;

            STATICTEXT 0 { NAME IDS_PBRT_MODE; ALIGN_LEFT; }
            COMBOBOX IDC_PBRT_MODE
            {
                //SIZE 100;
                SCALE_H;
                CHILDS
                {
                    IDC_PBRT_MODE_EXPORT_AND_RENDER, IDS_PBRT_MODE_EXPORT_AND_RENDER;
                    IDC_PBRT_MODE_EXPORT, IDS_PBRT_MODE_EXPORT;
                    IDC_PBRT_MODE_RENDER, IDS_PBRT_MODE_RENDER;
                }
            }

            STATICTEXT 0 { NAME IDS_PBRT_EXE; ALIGN_LEFT; }
            FILENAME IDC_PBRT_EXE { SCALE_H; }

            STATICTEXT 0 { NAME IDS_PBRT_SAMPLES; ALIGN_LEFT; }
            EDITNUMBERARROWS IDC_PBRT_SAMPLES { SCALE_H; }

            STATICTEXT 0 { NAME IDS_PBRT_INTENSITY; ALIGN_LEFT; }
            EDITNUMBERARROWS IDC_PBRT_INTENSITY { SCALE_H; }

            STATICTEXT 0 { NAME IDS_PBRT_LOGLEVEL; ALIGN_LEFT; }
            COMBOBOX IDC_PBRT_LOGLEVEL
            {
                SCALE_H;
                CHILDS
                {
                    IDC_PBRT_LOGLEVEL_DEBUG, IDS_PBRT_LOGLEVEL_DEBUG;
                    IDC_PBRT_LOGLEVEL_INFO, IDS_PBRT_LOGLEVEL_INFO;
                    IDC_PBRT_LOGLEVEL_WARNING, IDS_PBRT_LOGLEVEL_WARNING;
                    IDC_PBRT_LOGLEVEL_ERROR, IDS_PBRT_LOGLEVEL_ERROR;
                }
            }

            STATICTEXT 0 { }
            GROUP IDC_BUTTON_GROUP
            {
            	SCALE_H;
            	COLUMNS 1;
            }
        }

        TREEVIEW IDC_PBRT_LOG { HAS_HEADER; ALTERNATE_BG; FIXED_LAYOUT; RESIZE_HEADER; SCALE_V; SCALE_H; BORDER; }
    }
}
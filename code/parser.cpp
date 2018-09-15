#include "func.h"
#include "circuit.h"
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>


#define READ_FAILED 100
#define INVALID_FILE_FORMAT 200


#define _DEBUG_PARSER

void OABusRouter::Circuit::print_all()
{
    printf("DESIGN BOUNDAY (%d %d) (%d %d)\n\n", originX, originY, originX + width, originY + height);
    
    int i, j, numtracks, numlayers, numoffsets;
    int minpitch, maxpitch;
    int prev, cur;
    set<int> offsets;
    Layer* curL;

    numlayers = layers.size();
    printf("vertical ->     {"); 
    for(i=0; i < numlayers; i++)
    {
        curL = &layers[i];
        if(curL->direction == VERTICAL)
        {
            printf(" %s", curL->name.c_str());
        }
    }
    printf(" }\n");
    printf("horizontal ->   {"); 
    for(i=0; i < numlayers; i++)
    {
        curL = &layers[i];
        if(curL->direction == HORIZONTAL)
        {
            printf("%s ", curL->name.c_str());
        }
    }
    printf(" }\n\n");


    for(i=0; i < numlayers; i++)
    {
        offsets.clear();
        curL = &layers[i];
        numtracks = curL->trackOffsets.size();
        for(j=0; j < numtracks; j++)
        {
            offsets.insert(curL->trackOffsets[j]);
        }


        minpitch = INT_MAX;
        maxpitch = INT_MIN;
        numoffsets = offsets.size();

        printf("%s ->   {", curL->name.c_str());
        set<int>::iterator it = offsets.begin();
        while(it != offsets.end())
        {
            cur = *it;           
            //printf(" %d", *it);
            if(it != offsets.begin())
            {
                minpitch = min(minpitch, abs(prev - cur));
                maxpitch = max(maxpitch, abs(prev - cur));
            }
            prev = cur;
            it++;
        }
        printf(" #total %d min %d max %d }\n", numoffsets, minpitch, maxpitch );
    
    }

}

bool OABusRouter::Circuit::getParam(char* fileName){
    ifstream inputFile(fileName);
    bool runtimeFlag = false;
    bool alphaFlag = false;
    bool betaFlag = false;
    bool gammaFlag = false;
    bool deltaFlag = false;
    bool designFlag = false;
    bool epsilonFlag = false;
    string line = "";
    string delim = " ()";

    string input_file(fileName);

    size_t dot_point = input_file.find("input",0);
    size_t dot_slash = input_file.find_last_of("/\\");

    design = input_file.substr(dot_slash+1,dot_point-dot_slash-2);

    try{
        while(!inputFile.eof()){
            if(!getline(inputFile,line)) throw READ_FAILED;
            boost::char_separator<char> sep(delim.c_str());
            boost::tokenizer<boost::char_separator<char>> tokens(line, sep);
            typedef boost::tokenizer<boost::char_separator<char>>::iterator tokenIter;
            tokenIter iter = tokens.begin();

            if(*iter == "RUNTIME"){
                string str = *(++iter);
                this->runtime = atoi(str.c_str());//*(++iter));
                //cout << "Runtime : " << str << endl;
                runtimeFlag = true;
            }

            if(*iter == "ALPHA"){
                string str = *(++iter);
                this->alpha = atoi(str.c_str());//*(++iter));
                //cout << "Alpha : " << str << endl;
                alphaFlag = true;
            }

            if(*iter == "BETA"){
                string str = *(++iter);
                this->beta = atoi(str.c_str());//*(++iter));
                //cout << "Beta : " << str << endl;
                betaFlag = true;
            }

            if(*iter == "GAMMA"){
                string str = *(++iter);
                this->gamma = atoi(str.c_str());//*(++iter));
                //cout << "Gamma : " << str << endl;
                gammaFlag = true;
            }

            if(*iter == "DELTA"){
                string str = *(++iter);
                this->delta = atoi(str.c_str());//*(++iter));
                //cout << "Delta : " << str << endl;
                //.c_str());//*(tokens.begin() + 1);
                deltaFlag = true;
            }

            if(*iter == "EPSILON"){
                string str = *(++iter);
                this->epsilon = atoi(str.c_str());//*(++iter));
                //cout << "Epsilon : " << str << endl;
                //.c_str());//*(tokens.begin() + 1);
                epsilonFlag = true;
            }

            if(*iter == "DESIGN_BOUNDARY"){
                int llx, lly, urx, ury;
                string llxStr = *(++iter);
                string llyStr = *(++iter);
                string urxStr = *(++iter);
                string uryStr = *(++iter);

                llx = atoi(llxStr.c_str());
                lly = atoi(llyStr.c_str());
                urx = atoi(urxStr.c_str());
                ury = atoi(uryStr.c_str());
                
                this->originX = llx;
                this->originY = lly;
                this->width = abs(urx - llx);
                this->height = abs(ury - lly);
                
                designFlag = true;
            }

            if(runtimeFlag && alphaFlag && betaFlag && gammaFlag && epsilonFlag && designFlag){
                inputFile.close();
                return true;
            }
        }

        return false;
    }catch(int errorInfo){
        cout << "ERROR OCCURS. ERROR TYPE[" << errorInfo << "]" << endl;
        inputFile.close();
        return false;
    }

    inputFile.close();
    return false;
}

bool OABusRouter::Circuit::getLayerInfo(char* fileName){
    
    
    ifstream inputFile(fileName);
    string line = "";
    string delim = " ";
    bool flag = false;
    
    typedef boost::tokenizer<boost::char_separator<char>>::iterator tokenIter;
    string str ="";
    
    try{
        while(!inputFile.eof()){
            if(!getline(inputFile, line)) throw READ_FAILED;
            boost::char_separator<char> sep(delim.c_str());
            boost::tokenizer<boost::char_separator<char>> tokens(line, sep);
            tokenIter iter = tokens.begin();

            if(!flag){
                if(*iter == "LAYERS"){
                    flag = true;
                }else{
                    continue;
                }
            }else{
                if(*iter == "ENDLAYERS"){
                    inputFile.close();
#ifdef DEBUG_PARSER
                    for(auto& l : layers)
                    {
                        printf("%s %d\n", l.name.c_str(), l.id);
                    }
#endif


                    return true;
                }else{
                    
                    string layerName = *(iter++);
                    int direction = (*(iter++) == "vertical")? VERTICAL : HORIZONTAL;
                    str = *iter;
                    int spacing = atoi(str.c_str());

                    Layer layer;
                    layer.id = this->layers.size();
                    layer.name = layerName;
                    layer.direction = direction;
                    layer.spacing = spacing;
                    this->layers.push_back(layer);
                    this->layerHashMap.insert(make_pair(layerName,layer.id));
                    //this->layerHashMap[layerName] = layer.id;
                }
            }
        }

        inputFile.close();
        return false;
    }catch(int errorInfo){
        cout << "ERROR OCCURS. ERROR TYPE[" << errorInfo << "]" << endl;
        inputFile.close();
        return false;
    }

    inputFile.close();
    return false;
}

bool OABusRouter::Circuit::getTrackInfo(char* fileName){

    ifstream inputFile(fileName);
    string line = "";
    string delim = " ()";
    bool flag = false;

    typedef boost::tokenizer<boost::char_separator<char>>::iterator tokenIter;
    string str ="";
    
    try{
        while(!inputFile.eof()){
            if(!getline(inputFile, line)) throw READ_FAILED;
            
            boost::char_separator<char> sep(delim.c_str());
            boost::tokenizer<boost::char_separator<char>> tokens(line, sep);
            tokenIter iter = tokens.begin();

            if(!flag){
                if(*iter == "TRACKS"){
                    flag = true;
                }else{
                    continue;
                }
            }else{
                if(*iter == "ENDTRACKS"){
                    inputFile.close();

                    return true;
                }else{
                    string layerName = *iter++;
                    string llxStr = *iter++;
                    string llyStr = *iter++;
                    string urxStr = *iter++;
                    string uryStr = *iter++;
                    string widthStr = *iter++;
                    
                    //printf("Track coord %s %s %s %s\n", llxStr.c_str(), llyStr.c_str(), urxStr.c_str(), uryStr.c_str());
                    bool merge = false;
                    Track track;
                    track.id = this->tracks.size();
                    track.width = atoi(widthStr.c_str());
                    track.llx = atoi(llxStr.c_str());
                    track.lly = atoi(llyStr.c_str());
                    track.urx = atoi(urxStr.c_str());
                    track.ury = atoi(uryStr.c_str());
                    track.l = this->layerHashMap[layerName];
                    Layer* layer = &this->layers[this->layerHashMap[layerName]];
                    layer->min_width = min(layer->min_width,track.width);
                    assert(track.l == layer->id);

                    for(int i=0; i < tracks.size(); i++) {
                        Track* theTrack = &tracks[i];
                        if( theTrack->l != track.l )
                            continue;
                        else if ( theTrack->width != track.width )
                            continue;

                        if( theTrack->llx == track.urx && theTrack->lly == track.ury ) {
                            theTrack->llx == track.llx;
                            theTrack->lly = track.lly;
                            merge = true;
                        } else if ( theTrack->urx == track.llx && theTrack->ury == track.lly ) {
                            theTrack->urx == track.urx;
                            theTrack->ury == track.ury;
                            merge = true;
                        }
                    }

                    if ( merge == false ) {
                        layer->tracks.push_back(track.id);

                        if( layer->is_vertical() == true ) {
                            track.start.x = track.llx;;
                            track.start.y = track.lly;
                            track.end.x = track.urx;
                            track.end.y = track.ury;
                        } else {
                            track.start.x = track.llx;
                            track.start.y = track.lly;
                            track.end.x = track.urx;
                            track.end.y = track.ury;
                        }

                        track.offset = (layer->is_vertical())?track.llx:track.lly;
                        layer->trackOffsets.push_back(track.offset);

                        if(layer->offsets.find(track.offset) == layer->offsets.end())
                            layer->offsets.insert(track.offset); // (layer->is_vertical())?track.llx:track.lly);

                        //printf("Track Offset %d\n", track.offset);
                        this->tracks.push_back(track);
                    }
                }
            }
        }
        inputFile.close();
        return false;
    }catch(int errorInfo){
        cout << "ERROR OCCURS. ERROR TYPE[" << errorInfo << "]" << endl;
        inputFile.close();
        return false;
    }

}



bool OABusRouter::Circuit::getBusInfo(char* fileName){
    ifstream inputFile(fileName);
    string line = "";
    string delim = " ()";
    int count;
    bool flag = false;

    typedef boost::tokenizer<boost::char_separator<char>>::iterator tokenIter;
    string str ="";
    
    try
    {
        while(!inputFile.eof()){
            if(!getline(inputFile, line)) throw READ_FAILED;
            

            boost::char_separator<char> sep(delim.c_str());
            boost::tokenizer<boost::char_separator<char>> tokens(line, sep);
            tokenIter iter = tokens.begin();

            if(!flag){
                if(*iter == "BUSES"){
                    flag = true;
                }else{
                    continue;
                }
            }else{

                // BUSES INFO START
                if(*iter == "ENDBUSES"){
                    inputFile.close();
                    return true;
                }else{
                    // START BUS
                    if(*iter == "BUS"){
                        count = 0;
                        Bus bus;
                        bus.id = this->buses.size();
                        bus.name = *(++iter);

                        // Number of Bits
                        if(!getline(inputFile,line)) throw READ_FAILED;
                        tokens = boost::tokenizer<boost::char_separator<char>>(line, sep);
                        iter = tokens.begin();
                        string numBitStr = *iter++;
                        bus.numBits = atoi(numBitStr.c_str());
                        //cout << "numBits : " << numBitStr << endl;
                        
                        // Number of Pin Shapes
                        if(!getline(inputFile,line)) throw READ_FAILED;
                        tokens = boost::tokenizer<boost::char_separator<char>>(line, sep);
                        iter = tokens.begin();
                        string numPinShapeStr = *iter++;
                        bus.numPinShapes = atoi(numPinShapeStr.c_str());
                        //cout << "numPinShape : " << numPinShapeStr << endl;
                        

                        // MultiPin vector initialize
                        //vector<MultiPin> mps(bus.numPinShapes);
                        //for(int i=0; i < bus.numPinShapes; i++)
                        //    mps[i].busid = bus.id;

                        // Width Information
                        if(!getline(inputFile,line)) throw READ_FAILED;
                        tokens = boost::tokenizer<boost::char_separator<char>>(line, sep);
                        iter = tokens.begin();
                        
                        if(*iter++ == "WIDTH"){
                            str = *iter;
                            int numLayers = atoi(str.c_str());
                            for(int i=0; i < numLayers ; i++){
                                if(!getline(inputFile,line)) throw READ_FAILED;
                                tokens = boost::tokenizer<boost::char_separator<char>>(line, sep);
                                iter = tokens.begin();
                                string widthStr = *iter;
                                Layer* targetLayer = &this->layers[i];
                                bus.width[targetLayer->id] = atoi(widthStr.c_str());
                            }
                        }else{
                            throw INVALID_FILE_FORMAT;
                        }



                        // Bits Information
                        bool bitFlag = false;
                        int pin_count = 0;
                        Bit* targetBit;
                        while(true){
                            if(!getline(inputFile,line)) throw READ_FAILED;
                            tokens = boost::tokenizer<boost::char_separator<char>>(line, sep);
                            iter = tokens.begin();

                            if(*iter == "BIT"){
                                Bit bit;
                                bit.id = this->bits.size();
                                bit.seq = count++;
                                bit.busName = bus.name;
                                bit.name = *++iter;
                                //bit.name = bit.name + "_" + *++iter;
                                this->bits.push_back(bit);
                                this->bitHashMap[bit.name] = bit.id;
                                bitFlag = true;
                                targetBit = &this->bits[bit.id];//bit;
                                bus.bits.push_back(bit.id);
                               
                                // pin_count initialize 
                                pin_count = 0;

                                continue;
                            }else if(*iter == "ENDBIT"){
                                bitFlag = false;
                                continue;
                            }else if(*iter == "ENDBUS"){
                                break;
                            }


                            // Pin Information
                            if(bitFlag){
                                string layerStr = *iter++;
                                string llxStr = *iter++;
                                string llyStr = *iter++;
                                string urxStr = *iter++;
                                string uryStr = *iter++;

                                Pin pin;
                                pin.id = this->pins.size();
                                pin.bitName = targetBit->name;
                                pin.l = this->layerHashMap[layerStr];
                                int llx = atoi(llxStr.c_str());
                                int lly = atoi(llyStr.c_str());
                                int urx = atoi(urxStr.c_str());
                                int ury = atoi(uryStr.c_str());
                                //printf("Pin (%d %d) (%d %d)\n", llx, lly, urx, ury);
                                
                                pin.llx = llx;
                                pin.lly = lly;
                                pin.urx = urx;
                                pin.ury = ury;

                                //pin.boundary = Rect(Point(llx, lly), Point(urx, ury));
                                targetBit->pins.push_back(pin.id);
                                bus.llx = min(llx, bus.llx);
                                bus.lly = min(lly, bus.lly);
                                bus.urx = max(urx, bus.urx);
                                bus.ury = max(ury, bus.ury);
                                
                                this->pins.push_back(pin);

                                /*
                                mps[pin_count].pins.push_back(pin.id);
                                if( mps[pin_count].l == INT_MAX )
                                    mps[pin_count].l = pin.l;
                                else
                                    assert(mps[pin_count].l == pin.l);

                                pin_count++;
                                */
                            }
                        }

                        /*
                        // multi pin id mapping & store in ckt->multipins
                        for(int i=0; i < bus.numPinShapes; i++) {
                            mps[i].id = this->multipins.size();
                            
                            // determine pin align ( vertical or horizontal )
                            if( pins[mps[i].pins[0]].lly == pins[mps[i].pins[1]].lly )
                                mps[i].align = HORIZONTAL;
                            else if ( pins[mps[i].pins[0]].llx == pins[mps[i].pins[1]].llx )
                                mps[i].align = VERTICAL;

                            if( mps[i].align == this->layers[mps[i].l].direction )
                                mps[i].needVia = true;

                            assert ( mps[i].align != INT_MAX );

                            bus.multipins.push_back(mps[i].id);
                            this->multipins.push_back(mps[i]);
                        }
                        */
                        this->buses.push_back(bus);
                        this->busHashMap[bus.name] = bus.id;
                    
                    }
                    // END BUS
                }
            }
        }

        inputFile.close();
        return true;
    }catch(int errorInfo){
        cout << "INVALID INPUT FORMAT TYPE. ERROR CODE[" << errorInfo << "]" << endl;
        inputFile.close();
        return false;
    }

}


bool OABusRouter::Circuit::getObstacleInfo(char* fileName){
    ifstream inputFile(fileName);
    string line = "";
    string delim = " ()";
    bool flag = false;

    typedef boost::tokenizer<boost::char_separator<char>>::iterator tokenIter;
    string str ="";
    
    try{
        while(!inputFile.eof()){
            if(!getline(inputFile, line)) throw READ_FAILED;
            boost::char_separator<char> sep(delim.c_str());
            boost::tokenizer<boost::char_separator<char>> tokens(line, sep);
            tokenIter iter = tokens.begin();

            if(!flag){
                if(*iter == "OBSTACLES"){
                    flag = true;
                }else{
                    continue;
                }
            }else{
                if(*iter == "ENDOBSTACLES"){
                    inputFile.close();
                    return true;
                }else{
                    string layerStr = *iter++;
                    string llxStr = *iter++;
                    string llyStr = *iter++;
                    string urxStr = *iter++;
                    string uryStr = *iter++;
                    Obstacle obs;
                    obs.id = this->obstacles.size();
                    obs.l = this->layerHashMap[layerStr];
                    obs.llx = atoi(llxStr.c_str());
                    obs.lly = atoi(llyStr.c_str());
                    obs.urx = atoi(urxStr.c_str());
                    obs.ury = atoi(uryStr.c_str());
                    this->obstacles.push_back(obs);
                }
            }
        }
        
        if(!flag) throw INVALID_FILE_FORMAT;

        return true;
    }catch(int errorInfo){
        cout << "ERROR OCCURS. ERROR TYPE[" << errorInfo << "]" << endl;
        return false;
    }

}


bool OABusRouter::Circuit::read_iccad2018(char* fileName)
{


    if(!this->getParam(fileName)){
        cout << "[ERROR] Fail to get parameters" << endl;
        return false;
    }
    
    if(!this->getLayerInfo(fileName)){
        cout << "[ERROR] Fail to get layer information" << endl;
        return false;
    }

    if(!this->getTrackInfo(fileName)){
        cout << "[ERROR] Fail to get track information" << endl;
        return false;
    }

    if(!this->getBusInfo(fileName)){
        cout << "[ERROR] Fail to get bus information" << endl;
        return false;
    }

    if(!this->getObstacleInfo(fileName)){
        cout << "[ERROR] Fail to get obstacle information" << endl;
        return false;
    }
    
#ifdef DEBUG_PARSER
    for(auto& bus : buses) bus.print();
#endif


    for(int i=0; i<this->layers.size(); i++)
    {
        Layer* layer = &this->layers[i];
        sort(layer->trackOffsets.begin(), layer->trackOffsets.end(), [](int left, int right){
                return left < right;
                });
    }

    cout << "[INFO] Success parsing" << endl;
    return true;
}

/*
void OABusRouter::Rect::print()
{
    printf("Rect (%d %d) (%d %d)\n",this->ll.x, this->ll.y, this->ur.x, this->ur.y);
}

void OABusRouter::Point::print()
{
    printf("Point (%d %d)\n", this->x, this->y);
}

void OABusRouter::Layer::print(bool all = false)
{
    string dir = (this->direction == VERTICAL)? "vertical" : "horizontal";
    printf("\n ================================== \n");
    printf("(%d) Layer %s %s %d\n", this->id, this->name.c_str(), dir.c_str(), this->spacing);
    printf("    The number of tracks : %d\n", (int)this->trackOffsets.size());

    if(all)
    {
        

    }
    printf("\n ================================== \n");

}
void OABusRouter::Gcell::print()
{
    printf("(%d) Gcell %s (%d %d) (%d %d) Num Tracks %d\n", this->id, this->layer.c_str(), 
            this->boundary.ll.x, this->boundary.ll.y, this->boundary.ur.x, this->boundary.ur.y,
            (int)this->trackOffsets.size());       
}


void OABusRouter::Track::print()
{
    printf("(%d) Track %s (%d %d) (%d %d) %d\n", this->id, this->layer.c_str(), this->ll.x, this->ll.y, this->ur.x, this->ur.y, this->width);       
}

void OABusRouter::Pin::print()
{
    printf("(%d) Pin %s %s (%d %d) (%d %d)\n", this->id, this->bitName.c_str(), this->layer.c_str(), this->boundary.ll.x, this->boundary.ll.y,
            this->boundary.ur.x, this->boundary.ur.y);
}

void OABusRouter::Bit::print()
{
    printf("(%d) Bit %s [%s] # of Pins %d\n", this->id, this->name.c_str(), this->busName.c_str(), this->pins.size());
}
*/

void OABusRouter::Bus::print()
{
    printf("(%d) Bus %s [# of bits %d] [# of pin shapes %d]\n", this->id, this->name.c_str(), this->numBits, this->numPinShapes);
}
/*
void OABusRouter::Obstacle::print()
{
    printf("(%d) Obstacle %s (%d %d) (%d %d)\n", this->id, this->layer.c_str(), this->boundary.ll.x, this->boundary.ll.y,
            this->boundary.ur.x, this->boundary.ur.y);
}

*/

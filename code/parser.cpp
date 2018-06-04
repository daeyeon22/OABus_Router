#include "circuit.h"
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>


#define READ_FAILED 100
#define INVALID_FILE_FORMAT 200



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
    string delim = " ";


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
                cout << "Runtime : " << str << endl;
                runtimeFlag = true;
            }

            if(*iter == "ALPHA"){
                string str = *(++iter);
                this->alpha = atoi(str.c_str());//*(++iter));
                cout << "Alpha : " << str << endl;
                alphaFlag = true;
            }

            if(*iter == "BETA"){
                string str = *(++iter);
                this->beta = atoi(str.c_str());//*(++iter));
                cout << "Beta : " << str << endl;
                betaFlag = true;
            }

            if(*iter == "GAMMA"){
                string str = *(++iter);
                this->gamma = atoi(str.c_str());//*(++iter));
                cout << "Gamma : " << str << endl;
                gammaFlag = true;
            }

            if(*iter == "DELTA"){
                string str = *(++iter);
                this->delta = atoi(str.c_str());//*(++iter));
                cout << "Delta : " << str << endl;
                //.c_str());//*(tokens.begin() + 1);
                deltaFlag = true;
            }

            if(*iter == "EPSILON"){
                string str = *(++iter);
                this->epsilon = atoi(str.c_str());//*(++iter));
                cout << "Epsilon : " << str << endl;
                //.c_str());//*(tokens.begin() + 1);
                epsilonFlag = true;
            }

            if(*iter == "DESIGN_BOUNDARY"){
                string str = *(++iter);
                this->designBoundary.ll.x = atoi(str.c_str());
                str = *(++iter);
                this->designBoundary.ll.y = atoi(str.c_str());
                str = *(++iter);
                this->designBoundary.ur.x = atoi(str.c_str());
                str = *(++iter);
                this->designBoundary.ur.y = atoi(str.c_str());
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
                    this->layerHashMap[layerName] = layer.id;
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
                    Track track;
                    track.id = this->tracks.size();
                    track.width = atoi(widthStr.c_str());
                    track.ll = Point(atoi(llxStr.c_str()), atoi(llyStr.c_str()));
                    track.ur = Point(atoi(urxStr.c_str()), atoi(uryStr.c_str()));
                    track.layer = layerName;
                    this->tracks.push_back(track);
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
                        Bus bus;
                        bus.id = this->buses.size();
                        bus.name = *(++iter);

                        // Number of Bits
                        if(!getline(inputFile,line)) throw READ_FAILED;
                        tokens = boost::tokenizer<boost::char_separator<char>>(line, sep);
                        iter = tokens.begin();
                        string numBitStr = *iter++;
                        bus.numBits = atoi(numBitStr.c_str());
                        cout << "numBits : " << numBitStr << endl;
                        // Number of Pin Shapes
                        if(!getline(inputFile,line)) throw READ_FAILED;
                        tokens = boost::tokenizer<boost::char_separator<char>>(line, sep);
                        iter = tokens.begin();
                        string numPinShapeStr = *iter++;
                        bus.numPinShapes = atoi(numPinShapeStr.c_str());
                        cout << "numPinShape : " << numPinShapeStr << endl;
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
                                bus.width[targetLayer->name] = atoi(widthStr.c_str());
                            }
                        }else{
                            throw INVALID_FILE_FORMAT;
                        }



                        // Bits Information
                        bool bitFlag = false;
                        Bit* targetBit;
                        while(true){
                            if(!getline(inputFile,line)) throw READ_FAILED;
                            tokens = boost::tokenizer<boost::char_separator<char>>(line, sep);
                            iter = tokens.begin();

                            if(*iter == "BIT"){
                                Bit bit;
                                bit.id = this->bits.size();
                                bit.busName = bus.name;
                                bit.name = *++iter;
                                this->bits.push_back(bit);
                                this->bitHashMap[bit.name] = bit.id;
                                bitFlag = true;
                                targetBit = &bit;
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
                                pin.layer = layerStr;
                                pin.boundary.ll.x = atoi(llxStr.c_str());
                                pin.boundary.ll.y = atoi(llyStr.c_str());
                                pin.boundary.ur.x = atoi(urxStr.c_str());
                                pin.boundary.ur.y = atoi(uryStr.c_str());
                                targetBit->pins.push_back(pin.id);
                                this->pins.push_back(pin);
                            }
                        }

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
                    obs.layer = layerStr;
                    obs.boundary.ll.x = atoi(llxStr.c_str());
                    obs.boundary.ll.y = atoi(llyStr.c_str());
                    obs.boundary.ur.x = atoi(urxStr.c_str());
                    obs.boundary.ur.y = atoi(uryStr.c_str());
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
        cout << "Fail to get parameters" << endl;
        return false;
    }
    
    if(!this->getLayerInfo(fileName)){
        cout << "Fail to get layer information" << endl;
        return false;
    }

    if(!this->getTrackInfo(fileName)){
        cout << "Fail to get track information" << endl;
        return false;
    }

    if(!this->getBusInfo(fileName)){
        cout << "Fail to get bus information" << endl;
        return false;
    }

    if(!this->getObstacleInfo(fileName)){
        cout << "Fail to get obstacle information" << endl;
        return false;
    }
    

    for(auto& it : this->layers){
        it.print();
    }

    for(auto& it : this->tracks){
        it.print();
    }

    for(auto& it : this->buses){
        it.print();
    }

    for(auto& it : this->bits){
        it.print();
    }

    for(auto& it : this->pins){
        it.print();
    }

    for(auto& it : this->obstacles){
        it.print();
    }

    cout << "Success parsing" << endl;
    return true;
}

void OABusRouter::Rect::print()
{
    printf("Rect (%d %d) (%d %d)\n",this->ll.x, this->ll.y, this->ur.x, this->ur.y);
}

void OABusRouter::Point::print()
{
    printf("Point (%d %d)\n", this->x, this->y);
}

void OABusRouter::Layer::print()
{
    string dir = (this->direction == VERTICAL)? "vertical" : "horizontal";
    printf("(%d) Layer %s %s %d\n", this->id, this->name.c_str(), dir.c_str(), this->spacing);
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

void OABusRouter::Bus::print()
{
    printf("(%d) Bus %s [# of bits %d] [# of pin shapes %d]\n", this->id, this->name.c_str(), this->numBits, this->numPinShapes);
}

void OABusRouter::Obstacle::print()
{
    printf("(%d) Obstacle %s (%d %d) (%d %d)\n", this->id, this->layer.c_str(), this->boundary.ll.x, this->boundary.ll.y,
            this->boundary.ur.x, this->boundary.ur.y);
}


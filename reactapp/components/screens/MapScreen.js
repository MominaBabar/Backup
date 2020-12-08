import React from 'react';
import {Text, View,StyleSheet,TouchableOpacity,Image, Animated} from 'react-native';
import { RNCamera } from 'react-native-camera';
import CameraRoll from "@react-native-community/cameraroll";
import {connect} from 'react-redux';
import {addmedia,moverobot} from '../../actions/useraction';
import PropTypes from 'prop-types';




class MapScreen extends React.Component {
  state={
    granted:false,
    photo:{}
  }
  componentDidMount(){
    console.log(this.props.user)
    //this.props.getalarms(this.props.route.params.user.user.machineID._id);
   
  }
  onmove(dir){
    this.setState({move:dir});
    setTimeout(()=>{
        console.log(this.props.route.params.user.user.machineID._id,this.state.move)
        this.props.moverobot(this.props.route.params.user.user.machineID._id,this.state.move);
      }, 100);
   
  }
     
     
      render(){
      return (
        <View style={styles.container}>
         
          <RNCamera
          ref={(ref) => {
            camera = ref;
          }}
          style={styles.preview}
          type={RNCamera.Constants.Type.back}
          flashMode={RNCamera.Constants.FlashMode.off}
          androidCameraPermissionOptions={{
            title: 'Permission to use camera',
            message: 'We need your permission to use your camera',
            buttonPositive: 'Ok',
            buttonNegative: 'Cancel',
          }}
         
          androidStoragePermissionOptions={{
          title: 'Permission to use Storage',
          message: 'We need your permission to use your audio',
          buttonPositive: 'Ok',
          buttonNegative: 'Cancel',}}
          androidRecordAudioPermissionOptions={{
            title: 'Permission to use audio recording',
            message: 'We need your permission to use your audio',
            buttonPositive: 'Ok',
            buttonNegative: 'Cancel',
          }}
         
        />
        <View  style={{top:240, left:25, fontSize: 3}}>
     <Text style={{fontSize: 17, color: '#FFFFFF'}}>Cleaning....</Text>
     <View style={styles.btnstyle}>
     <TouchableOpacity style={styles.btn} activeOpacity={0.5} onPress={() => this.onmove('w')}>
        <Image source={require('./../../assets/up-arrow.png')}  style={styles.img}/>
    </TouchableOpacity>
    </View>
     </View>
     <View style={styles.progressBar}>
       <Animated.View style={[StyleSheet.absoluteFill], {backgroundColor: "#87cefa", width: "50%"}}/>
       </View>
     <View style={styles.percentText}>
       <Text style={{fontSize: 17,  color: '#FFFFFF'}}>50%</Text>
     </View>
     <View style={styles.btnstyle}>
     <TouchableOpacity style={styles.btn} activeOpacity={0.5} onPress={() => this.onmove('w')}>
        <Image source={require('./../../assets/up-arrow.png')}  style={styles.img}/>
    </TouchableOpacity>
    </View>
    <View style={styles.btnstyle}>
     <TouchableOpacity style={{top: 100}} activeOpacity={0.5} onPress={() => this.onmove('f')}>
        <Image source={require('./../../assets/down-arrow.png')}  style={styles.img}/>
    </TouchableOpacity>
    </View>
    <View style={{}}>
     <TouchableOpacity style={{top: 60, flexDirection: 'row', left: 60}} activeOpacity={0.5} onPress={() => this.onmove('a')}>
        <Image source={require('./../../assets/back.png')}  style={styles.img}/>
    </TouchableOpacity>
    </View>
    <View style={{}}>
     <TouchableOpacity style={{top: 2, flexDirection: 'row', left: 173}} activeOpacity={0.5} onPress={() => this.onmove('x')}>
        <Image source={require('./../../assets/multimedia.png')}  style={styles.playimg}/>
    </TouchableOpacity>
    </View>
    <View style={{}}>
     <TouchableOpacity style={{top: -70, flexDirection: 'row', left: 200}} activeOpacity={0.5} onPress={() => this.onmove('d')}>
        <Image source={require('./../../assets/send.png')}  style={styles.img}/>
    </TouchableOpacity>
        </View>
      </View>
    );
      }
 
}
MapScreen.propTypes = {
  addmedia: PropTypes.func.isRequired,
  moverobot: PropTypes.func.isRequired,
   
}

const mapStateToProps = (state) => ({
  user: state.user
});
export default connect(mapStateToProps, {addmedia,moverobot})(MapScreen);
const styles = StyleSheet.create({
  btnstyle:{
    flex: 1,
    alignItems: 'center',
    justifyContent: 'center',
  },
btn:{
   top: 175,
   height: 60,
   width: 60
},
img: {
  width:60,
  height: 60
},
playimg: {
  width:70,
  height: 70
},
    container: {
      position:'absolute',
      top: 5,
      fontSize: 30,
      justifyContent: "center",
      backgroundColor: '#FFFFFF',
    },
    percentText: {
      fontSize: 30,
      top: 260,
       flex:1,
      alignItems: 'center',
     
    },
    progressBar: {
      height: 30,
      width: '90%',
      flexDirection:'row',
      backgroundColor: '#FFFFFF',
      borderColor: '#000',
      borderWidth: 2,
      borderRadius: 5,
      top: 245,
      left: 20,
    },
  container: {
    flex: 1,
    flexDirection: 'column',
    backgroundColor: 'black',
  },
  preview: {
    flex: 1,
    justifyContent: 'flex-end',
    alignItems: 'center',
  },
  capture: {
    flex: 0,
    backgroundColor: '#fff',
    borderRadius: 5,
    padding: 15,
    paddingHorizontal: 20,
    alignSelf: 'center',
    margin: 20,
  },
  image:{
    height:40,
    width:40
  }
});



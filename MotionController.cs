using UnityEngine;
using System.IO.Ports;
using System.Linq;

public class MotionController
{
	private SerialPort serial;
	private Vector3 rotation;
	private bool leftButton = false;
	private bool rightButton = false;
	
    // Port is the name of the port on which the controller is connected
	public MotionController(string port)
	{
		serial = new SerialPort(port, 9600);
        serial.ReadTimeout = 200;

        serial.Open();
	}

    ~MotionController()
    {
        serial.Close();
    }
	
    // Gets fresh data from the controller, parses them and stores them
	public void RefreshData()
	{
        string data = serial.ReadLine();
        if (!data.StartsWith('L')) return;
        
        var split = data.Split(',');

        leftButton = ((int) ParsePiece(split[0])) == 1;
        rightButton = ((int) ParsePiece(split[1])) == 1;
        // Juggle the values and invert them
        rotation.x = ParsePiece(split[3]);
        rotation.y = ParsePiece(split[2]);
        rotation.z = -ParsePiece(split[4]);
	}

    private float ParsePiece(string piece)
    {
        piece = new string(piece.Where(c => char.IsDigit(c) || c == '.' || c == '-').ToArray());
        return float.Parse(piece);
    }

    // Getter for rotation
    public Vector3 GetRotation()
    {
        return rotation;
    }

    // Getter for left button
    public bool GetLeftButton()
    {
        return leftButton;
    }

    // Getter for right button
    public bool GetRightButton()
    {
        return rightButton;
    }
}
